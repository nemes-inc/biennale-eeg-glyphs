//! **eeg-hub** — Multi-headband EEG viewer with TCP streaming to GPU inference server.
//!
//! Connects to up to 4 Muse headbands via BLE, displays raw EEG, streams
//! EEGM frames to a remote inference server, and overlays reconstructed
//! results as they arrive (delayed — ZUNA inference is ~4-5× slower than
//! real-time).
//!
//! Usage:
//! ```text
//! eeg-hub --server 192.168.1.50:9100
//! eeg-hub --server 192.168.1.50:9100 --headbands 2
//! eeg-hub --no-server                  # BLE viewer only, no inference
//! ```

mod muse_multi;
mod tcp_client;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use clap::Parser;
use eframe::egui;
use egui::RichText;
use egui_plot::{Legend, Line, Plot, PlotPoints};
use muse_rs::eegm::EegmFrame;

use muse_multi::{CHANNELS_PER_HEADBAND, MAX_HEADBANDS};

/// Rolling buffer capacity per channel.
const MAX_POINTS: usize = 8192;

/// Muse 4-channel electrode names.
const CHANNEL_NAMES: [&str; 4] = ["TP9", "AF7", "AF8", "TP10"];

/// Channel colours.
const CHANNEL_COLORS: [egui::Color32; 4] = [
    egui::Color32::from_rgb(0, 200, 100),   // TP9 green
    egui::Color32::from_rgb(0, 128, 255),   // AF7 blue
    egui::Color32::from_rgb(255, 128, 0),   // AF8 orange
    egui::Color32::from_rgb(200, 50, 200),  // TP10 purple
];

const COLOR_RECONSTRUCTED: egui::Color32 = egui::Color32::from_rgb(255, 48, 48);

#[derive(Parser, Debug)]
#[command(name = "eeg-hub", about = "Multi-headband EEG viewer + inference streaming")]
struct Args {
    /// Inference server address (e.g. 192.168.1.50:9100).
    #[arg(long)]
    server: Option<String>,

    /// Disable inference server connection (BLE viewer only).
    #[arg(long)]
    no_server: bool,

    /// Number of headbands to scan for (1–4).
    #[arg(long, default_value_t = 1)]
    headbands: usize,

    /// BLE scan timeout per headband in seconds.
    #[arg(long, default_value_t = 30)]
    scan_timeout: u64,

    /// Muse device name prefix for BLE scan filtering.
    #[arg(long, default_value = "Muse")]
    name_prefix: String,

    /// Rolling buffer size per channel.
    #[arg(long, default_value_t = MAX_POINTS)]
    max_points: usize,
}

/// Shared state between BLE tasks, TCP client, and the egui UI.
pub struct HeadbandState {
    /// Per-headband, per-channel raw sample buffers.
    pub raw_buffers: Vec<Vec<Vec<f32>>>,     // [headband][channel][samples]
    /// Per-headband, per-channel reconstructed sample buffers.
    pub recon_buffers: Vec<Vec<Vec<f32>>>,   // [headband][channel][samples]
    /// Per-headband frame counters.
    pub raw_frames: Vec<u64>,
    pub recon_frames: Vec<u64>,
    /// Per-headband connection status.
    pub connected: Vec<bool>,
    pub status: Vec<String>,
    /// Inference server status.
    pub server_connected: bool,
    pub server_status: String,
    /// Max points per channel buffer.
    pub max_points: usize,
}

impl HeadbandState {
    fn new(max_points: usize) -> Self {
        Self {
            raw_buffers: vec![vec![Vec::new(); CHANNELS_PER_HEADBAND]; MAX_HEADBANDS],
            recon_buffers: vec![vec![Vec::new(); CHANNELS_PER_HEADBAND]; MAX_HEADBANDS],
            raw_frames: vec![0; MAX_HEADBANDS],
            recon_frames: vec![0; MAX_HEADBANDS],
            connected: vec![false; MAX_HEADBANDS],
            status: vec!["Idle".to_string(); MAX_HEADBANDS],
            server_connected: false,
            server_status: "Not connected".to_string(),
            max_points,
        }
    }

    /// Push a raw EEG frame for a headband.
    pub fn push_raw(&mut self, headband_id: usize, channels: Vec<Vec<f32>>) {
        for (ch, samples) in channels.iter().enumerate() {
            if ch < CHANNELS_PER_HEADBAND {
                self.raw_buffers[headband_id][ch].extend_from_slice(samples);
                let len = self.raw_buffers[headband_id][ch].len();
                if len > self.max_points {
                    self.raw_buffers[headband_id][ch].drain(0..len - self.max_points);
                }
            }
        }
        self.raw_frames[headband_id] += 1;
    }

    /// Push a reconstructed frame from the inference server.
    pub fn push_reconstructed(&mut self, headband_id: usize, frame: EegmFrame) {
        let n_ch = frame.n_channels as usize;
        // Ensure recon_buffers has enough channels
        while self.recon_buffers[headband_id].len() < n_ch {
            self.recon_buffers[headband_id].push(Vec::new());
        }
        for ch in 0..n_ch {
            let data = frame.channel_data(ch);
            self.recon_buffers[headband_id][ch].extend_from_slice(data);
            let len = self.recon_buffers[headband_id][ch].len();
            if len > self.max_points {
                self.recon_buffers[headband_id][ch].drain(0..len - self.max_points);
            }
        }
        self.recon_frames[headband_id] += 1;
    }
}

/// Application state for the egui viewer.
struct EegHubApp {
    state: Arc<Mutex<HeadbandState>>,
    n_headbands: usize,
    show_raw: bool,
    show_recon: bool,
    outbound_tx: Option<tcp_client::OutboundTx>,
}

impl EegHubApp {
    fn new(
        state: Arc<Mutex<HeadbandState>>,
        n_headbands: usize,
        outbound_tx: Option<tcp_client::OutboundTx>,
    ) -> Self {
        Self {
            state,
            n_headbands: n_headbands.min(MAX_HEADBANDS),
            show_raw: true,
            show_recon: true,
            outbound_tx,
        }
    }
}

impl eframe::App for EegHubApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Auto-repaint for live data
        ctx.request_repaint_after(Duration::from_millis(33)); // ~30 fps

        let (
            raw_buf_snapshot,
            recon_buf_snapshot,
            statuses,
            connected,
            raw_frames,
            recon_frames,
            server_status,
            server_connected,
        ) = {
            let st = self.state.lock().unwrap();
            (
                st.raw_buffers.clone(),
                st.recon_buffers.clone(),
                st.status.clone(),
                st.connected.clone(),
                st.raw_frames.clone(),
                st.recon_frames.clone(),
                st.server_status.clone(),
                st.server_connected,
            )
        };

        // ── Top toolbar ──────────────────────────────────────────────────
        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new("EEG Hub").strong());
                ui.separator();
                ui.checkbox(&mut self.show_raw, "Raw");
                ui.checkbox(&mut self.show_recon, "Reconstructed");
                ui.separator();

                // Server status
                if server_connected {
                    ui.colored_label(
                        egui::Color32::from_rgb(80, 200, 120),
                        format!("● Server: {server_status}"),
                    );
                } else {
                    ui.label(
                        RichText::new(format!("○ Server: {server_status}")).weak(),
                    );
                }

                ui.separator();

                // Per-headband status
                for h in 0..self.n_headbands {
                    let color = if connected[h] {
                        egui::Color32::from_rgb(80, 200, 120)
                    } else {
                        egui::Color32::GRAY
                    };
                    ui.colored_label(color, format!("Band {}: {}", h, statuses[h]));
                    if h < self.n_headbands - 1 {
                        ui.separator();
                    }
                }
            });
        });

        // ── Main area: one plot panel per headband ───────────────────────
        egui::CentralPanel::default().show(ctx, |ui| {
            let available = ui.available_size();
            let panel_height = available.y / self.n_headbands.max(1) as f32;

            for h in 0..self.n_headbands {
                ui.group(|ui| {
                    ui.set_min_height(panel_height - 8.0);

                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(format!("Headband {h}"))
                                .strong()
                                .size(14.0),
                        );
                        ui.label(format!(
                            "raw: {} frames  recon: {} frames",
                            raw_frames[h], recon_frames[h]
                        ));
                    });

                    let plot = Plot::new(format!("headband_{h}"))
                        .height(panel_height - 40.0)
                        .legend(Legend::default())
                        .show_axes(true);

                    plot.show(ui, |plot_ui| {
                        // Raw traces
                        if self.show_raw {
                            for ch in 0..CHANNELS_PER_HEADBAND {
                                let buf = &raw_buf_snapshot[h][ch];
                                if buf.is_empty() {
                                    continue;
                                }
                                let offset = ch as f64 * 200.0; // vertical separation
                                let points: PlotPoints = buf
                                    .iter()
                                    .enumerate()
                                    .map(|(i, &v)| [i as f64, v as f64 + offset])
                                    .collect();
                                plot_ui.line(
                                    Line::new(points)
                                        .name(format!("{} raw", CHANNEL_NAMES[ch]))
                                        .color(CHANNEL_COLORS[ch])
                                        .width(1.0),
                                );
                            }
                        }

                        // Reconstructed traces
                        if self.show_recon {
                            let recon = &recon_buf_snapshot[h];
                            for ch in 0..recon.len().min(CHANNELS_PER_HEADBAND) {
                                let buf = &recon[ch];
                                if buf.is_empty() {
                                    continue;
                                }
                                let offset = ch as f64 * 200.0;
                                let points: PlotPoints = buf
                                    .iter()
                                    .enumerate()
                                    .map(|(i, &v)| [i as f64, v as f64 + offset])
                                    .collect();
                                plot_ui.line(
                                    Line::new(points)
                                        .name(format!("{} recon", CHANNEL_NAMES[ch]))
                                        .color(COLOR_RECONSTRUCTED)
                                        .width(1.5),
                                );
                            }
                        }
                    });
                });
            }
        });
    }
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    let n_headbands = args.headbands.min(MAX_HEADBANDS).max(1);

    let state = Arc::new(Mutex::new(HeadbandState::new(args.max_points)));

    // ── Tokio runtime for BLE + TCP tasks ────────────────────────────────
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .expect("tokio runtime");

    // ── Start inference server TCP client ─────────────────────────────────
    let outbound_tx = if !args.no_server {
        if let Some(ref addr) = args.server {
            let (tx, rx) = tcp_client::outbound_channel();
            let state_tcp = Arc::clone(&state);
            let addr = addr.clone();
            let n_hb = n_headbands as u32;
            rt.spawn(async move {
                let _ = tcp_client::spawn_tcp_client(addr, n_hb, 256, state_tcp, rx).await;
            });
            Some(tx)
        } else {
            log::warn!("No --server specified and --no-server not set; running without inference");
            None
        }
    } else {
        None
    };

    // ── Start BLE connections ────────────────────────────────────────────
    // TODO: Multi-headband scanning needs work — currently each MuseClient
    // grabs the first Muse it finds.  Phase 2 will implement device-specific
    // targeting (by name suffix or MAC).
    let _ble_handles: Vec<_> = (0..n_headbands)
        .map(|h| {
            let st = Arc::clone(&state);
            let prefix = args.name_prefix.clone();
            let timeout = args.scan_timeout;
            rt.spawn(async move {
                muse_multi::spawn_headband_task(h, st, prefix, timeout)
            })
        })
        .collect();

    // ── egui window ──────────────────────────────────────────────────────
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_title("EEG Hub"),
        ..Default::default()
    };

    let state_ui = Arc::clone(&state);
    eframe::run_native(
        "EEG Hub",
        native_options,
        Box::new(move |_cc| {
            Ok(Box::new(EegHubApp::new(state_ui, n_headbands, outbound_tx)))
        }),
    )
    .expect("eframe");
}
