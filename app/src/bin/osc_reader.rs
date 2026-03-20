//! OSC reader — receives Mind Monitor's OSC stream and logs to CSV.
//!
//! Mind Monitor streams EEG + derived data over OSC (UDP).  This tool
//! listens on a configurable port, parses every OSC message, prints a
//! summary to stdout, and appends every message to a timestamped CSV file
//! for offline comparison with muse-rs captures.
//!
//! # Usage
//!
//! ```bash
//! cargo run --bin osc-reader -- --port 5000 --output mind-monitor.csv
//! ```
//!
//! In Mind Monitor: Settings → OSC Stream Target IP = <this machine's IP>,
//! Port = 5000, then tap "Start Streaming".

use std::fs::{File, OpenOptions};
use std::io::Write;
use std::net::UdpSocket;
use std::time::Instant;

use clap::Parser;
use rosc::{OscMessage, OscPacket, OscType};

// ── CLI ──────────────────────────────────────────────────────────────────────

#[derive(Parser)]
#[command(name = "osc-reader", about = "Receive Mind Monitor OSC and log to CSV")]
struct Cli {
    /// UDP port to listen on (must match Mind Monitor's OSC port setting).
    #[arg(short, long, default_value_t = 5000)]
    port: u16,

    /// Output CSV file path.
    #[arg(short, long, default_value = "mind-monitor.csv")]
    output: String,

    /// Bind address (use 0.0.0.0 to accept from any interface).
    #[arg(short, long, default_value = "0.0.0.0")]
    bind: String,
}

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Format a single OSC argument as a compact string.
fn fmt_arg(a: &OscType) -> String {
    match a {
        OscType::Int(v) => format!("{v}"),
        OscType::Float(v) => format!("{v:.6}"),
        OscType::Double(v) => format!("{v:.6}"),
        OscType::String(v) => format!("\"{v}\""),
        OscType::Long(v) => format!("{v}"),
        OscType::Bool(v) => format!("{v}"),
        OscType::Nil => "nil".to_string(),
        _ => format!("{a:?}"),
    }
}

/// Write CSV header.
fn write_header(f: &mut File) {
    writeln!(f, "elapsed_s,address,arg_count,args").unwrap();
}

/// Write one OSC message as a CSV row.
fn write_row(f: &mut File, elapsed: f64, msg: &OscMessage) {
    let args_str: Vec<String> = msg.args.iter().map(fmt_arg).collect();
    writeln!(
        f,
        "{elapsed:.4},{},{},{}",
        msg.addr,
        msg.args.len(),
        args_str.join(";"),
    )
    .unwrap();
}

/// Recursively handle an OSC packet (may contain bundles of messages).
fn handle_packet(
    packet: OscPacket,
    start: Instant,
    file: &mut File,
    counters: &mut MessageCounters,
) {
    match packet {
        OscPacket::Message(msg) => {
            let elapsed = start.elapsed().as_secs_f64();
            let args_str: Vec<String> = msg.args.iter().map(fmt_arg).collect();

            // Track message counts for periodic summary
            counters.total += 1;
            counters.track(&msg.addr);

            // Print to stdout (compact one-liner)
            println!(
                "[{elapsed:8.3}s] {:<40} {}",
                msg.addr,
                args_str.join(", "),
            );

            // Append to CSV
            write_row(file, elapsed, &msg);
        }
        OscPacket::Bundle(bundle) => {
            for p in bundle.content {
                handle_packet(p, start, file, counters);
            }
        }
    }
}

// ── Message counters for summary ─────────────────────────────────────────────

struct MessageCounters {
    total: usize,
    by_address: std::collections::HashMap<String, usize>,
    last_summary: Instant,
}

impl MessageCounters {
    fn new() -> Self {
        Self {
            total: 0,
            by_address: std::collections::HashMap::new(),
            last_summary: Instant::now(),
        }
    }

    fn track(&mut self, addr: &str) {
        *self.by_address.entry(addr.to_string()).or_insert(0) += 1;
    }

    /// Print a summary every N seconds, return true if printed.
    fn maybe_print_summary(&mut self, interval_secs: f64) -> bool {
        if self.last_summary.elapsed().as_secs_f64() >= interval_secs {
            eprintln!("\n--- Summary ({} messages total) ---", self.total);
            let mut sorted: Vec<_> = self.by_address.iter().collect();
            sorted.sort_by(|a, b| b.1.cmp(a.1));
            for (addr, count) in &sorted {
                eprintln!("  {:<40} {count:>6}", addr);
            }
            eprintln!("---\n");
            self.last_summary = Instant::now();
            true
        } else {
            false
        }
    }
}

// ── Main ─────────────────────────────────────────────────────────────────────

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();

    let addr = format!("{}:{}", cli.bind, cli.port);
    let socket = UdpSocket::bind(&addr)?;
    eprintln!("Listening for OSC on {addr}");
    eprintln!("Output CSV: {}", cli.output);
    eprintln!("Waiting for Mind Monitor to connect…\n");

    let mut file = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(&cli.output)?;
    write_header(&mut file);

    let start = Instant::now();
    let mut counters = MessageCounters::new();
    let mut buf = [0u8; 65536]; // OSC packets are typically small

    loop {
        let (len, src) = socket.recv_from(&mut buf)?;

        // First packet — announce source
        if counters.total == 0 {
            eprintln!("Receiving from {src}\n");
        }

        match rosc::decoder::decode_udp(&buf[..len]) {
            Ok((_, packet)) => {
                handle_packet(packet, start, &mut file, &mut counters);
                file.flush()?;
            }
            Err(e) => {
                eprintln!("[WARN] Failed to decode OSC packet ({len} bytes): {e}");
            }
        }

        // Print periodic summary to stderr every 10 seconds
        counters.maybe_print_summary(10.0);
    }
}
