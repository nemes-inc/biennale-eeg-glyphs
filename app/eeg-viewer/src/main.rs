//! Muse-first EEG viewer: **Bluetooth Muse** live plot by default; **stdin** / **TCP** for EEGF/EEGD pipes.
//!
//! Binary frame formats (little-endian):
//!
//! **Single** (magic `0x45454746` ASCII `EEGF`):
//! - `u32` magic, `u32` `n_channels`, `u32` `n_samples`, channel-major `f32` payload
//!
//! **Dual** (magic `0x45454744` ASCII `EEGD`): original block + reconstructed block (same layout each).

mod muse_ble;

use std::fs;
use std::io::{self, BufRead, BufReader, Read};
use std::net::{SocketAddr, TcpListener};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use muse_ble::MuseLiveHandle;

use clap::Parser;
use eframe::egui;
use egui::RichText;
use egui_plot::{Legend, Line, Plot, PlotPoints};

const MAGIC_SINGLE: u32 = 0x4545_4746; // "EEGF"
const MAGIC_DUAL: u32 = 0x4545_4744; // "EEGD"

/// Under `--zuna-dir`; must match `run_fif_pipeline.py` default `--work-dir`.
const ZUNA_PIPELINE_WORK_DIR: &str = "zuna_pipeline_run";

/// Muse 4-channel electrode names.
const CHANNEL_NAMES_MUSE_4: [&str; 4] = ["TP9", "AF7", "AF8", "TP10"];

/// ZUNA channel upsampling preset.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
enum ZunaPreset {
    #[default]
    None,
    Ch8,
    Ch10,
}

/// Plot input source.
#[derive(Clone, Copy, PartialEq, Eq)]
enum InputKind {
    MuseBle,
    Stdin,
    Tcp,
    None,
}

impl ZunaPreset {
    fn as_cli(self) -> &'static str {
        match self {
            ZunaPreset::None => "none",
            ZunaPreset::Ch8 => "8ch",
            ZunaPreset::Ch10 => "10ch",
        }
    }

    fn label(self) -> &'static str {
        match self {
            ZunaPreset::None => "4ch (none)",
            ZunaPreset::Ch8 => "8ch 10–20",
            ZunaPreset::Ch10 => "10ch 10–20",
        }
    }
}

#[derive(Parser, Debug)]
#[command(name = "eeg-viewer")]
struct Args {
    /// Rolling buffer size per channel.
    #[arg(long, default_value_t = 8192)]
    max_points: usize,

    /// Read EEGF/EEGD from stdin; disables Muse BLE preview.
    #[arg(long)]
    stdin: bool,

    /// Listen for EEGF/EEGD frames on a TCP socket; disables Muse BLE preview.
    #[arg(long)]
    tcp: Option<String>,

    /// Disable Muse Bluetooth live preview.
    #[arg(long)]
    no_muse_ble: bool,

    /// Path to `services/zuna` directory.
    #[arg(
        long,
        default_value = concat!(env!("CARGO_MANIFEST_DIR"), "/../../services/zuna")
    )]
    zuna_dir: PathBuf,

    /// Path to the `muse-record-fif` binary.
    #[arg(
        long,
        default_value = concat!(env!("CARGO_MANIFEST_DIR"), "/../target/release/muse-record-fif")
    )]
    muse_record_bin: PathBuf,
}

#[derive(Default)]
struct SharedState {
    channels_orig: Vec<Vec<f32>>,
    channels_recon: Vec<Vec<f32>>,
    frames_received: u64,
    last_error: Option<String>,
    /// True once the `--tcp` listener accepts a client.
    tcp_client_connected: bool,
    /// True while Muse BLE is connected and streaming.
    muse_ble_streaming: bool,
    /// True after a BLE connect failure; toolbar shows Retry.
    muse_ble_retry_pending: bool,
    /// Channel names from `fif_pair_to_eegd.py` NAMES: header.
    channel_names: Option<Vec<String>>,
}

struct EegViewerApp {
    state: Arc<Mutex<SharedState>>,
    show_live: bool,
    /// Draw original trace in blue.
    show_original: bool,
    /// Draw reconstructed trace in red when EEGD data is present.
    show_reconstructed: bool,
    /// Duration in seconds for Record / Preprocess.
    record_secs: u32,
    zuna_dir: PathBuf,
    muse_record_bin: PathBuf,
    job_busy: bool,
    job_log: String,
    job_rx: Option<Receiver<JobEvent>>,
    /// Toolbar status line.
    muse_status: String,
    /// ZUNA preset for Preprocess.
    zuna_preset: ZunaPreset,
    /// Muse BLE handle; paused during Record / Preprocess.
    muse_live: Arc<Mutex<Option<MuseLiveHandle>>>,
    max_points: usize,
    input_kind: InputKind,
    job_started_at: Option<Instant>,
    job_last_duration: Option<Duration>,
    pipeline_step: Option<(u8, u8)>,
    pipeline_label: String,
    resume_muse_pending: bool,
}

/// Job events from background Record / Preprocess threads.
enum JobEvent {
    LogLine(String),
    Step {
        n: u8,
        total: u8,
        label: String,
    },
    Done {
        ok: bool,
        pipeline_loaded: bool,
        preprocess: bool,
    },
}

fn resolve_input_kind(args: &Args) -> InputKind {
    if args.tcp.is_some() {
        InputKind::Tcp
    } else if args.stdin {
        InputKind::Stdin
    } else if args.no_muse_ble {
        InputKind::None
    } else {
        InputKind::MuseBle
    }
}

fn stop_muse_if_any(muse: &Arc<Mutex<Option<MuseLiveHandle>>>) {
    if let Some(h) = muse.lock().unwrap().take() {
        h.stop_and_join();
        thread::sleep(Duration::from_millis(400));
    }
}

const COLOR_ORIGINAL: egui::Color32 = egui::Color32::from_rgb(0, 128, 255);
const COLOR_RECONSTRUCTED: egui::Color32 = egui::Color32::from_rgb(255, 48, 48);

impl EegViewerApp {
    fn poll_job(&mut self) {
        let Some(rx) = self.job_rx.take() else {
            return;
        };
        loop {
            match rx.try_recv() {
                Ok(JobEvent::LogLine(s)) => {
                    self.job_log.push_str(&s);
                    if !s.ends_with('\n') {
                        self.job_log.push('\n');
                    }
                }
                Ok(JobEvent::Step { n, total, label }) => {
                    self.pipeline_step = Some((n, total));
                    self.pipeline_label = label;
                }
                Ok(JobEvent::Done {
                    ok,
                    pipeline_loaded,
                    preprocess,
                }) => {
                    if let Some(t0) = self.job_started_at.take() {
                        self.job_last_duration = Some(t0.elapsed());
                    }
                    self.job_busy = false;
                    self.job_rx = None;
                    self.resume_muse_pending = ok
                        && pipeline_loaded
                        && preprocess
                        && self.input_kind == InputKind::MuseBle;
                    if preprocess && ok && pipeline_loaded {
                        // Show static comparison; live BLE stays paused.
                        self.show_live = false;
                        self.show_original = true;
                        self.show_reconstructed = true;
                    }
                    if preprocess {
                        self.muse_status = if ok {
                            if pipeline_loaded {
                                "Muse: OK — recording + ZUNA pipeline finished; comparison loaded"
                                    .to_string()
                            } else {
                                "Muse: OK — recording + ZUNA pipeline finished".to_string()
                            }
                        } else {
                            muse_status_from_job_log(&self.job_log, false)
                        };
                    } else {
                        self.muse_status = muse_status_from_job_log(&self.job_log, ok);
                        self.pipeline_step = None;
                        self.pipeline_label.clear();
                    }
                    return;
                }
                Err(TryRecvError::Empty) => {
                    self.job_rx = Some(rx);
                    return;
                }
                Err(TryRecvError::Disconnected) => {
                    self.job_busy = false;
                    self.job_started_at = None;
                    self.job_log.push_str("Job channel closed.\n");
                    return;
                }
            }
        }
    }

    fn resume_muse_ble(&mut self) {
        if !self.resume_muse_pending || self.input_kind != InputKind::MuseBle {
            return;
        }
        self.resume_muse_pending = false;
        self.show_live = true;
        let h = muse_ble::spawn_muse_ble_thread(Arc::clone(&self.state), self.max_points);
        *self.muse_live.lock().unwrap() = Some(h);
        self.muse_status = "Muse BLE: resumed".to_string();
    }

    /// Restart Muse BLE after a scan/connect timeout.
    fn retry_muse_ble(&mut self) {
        if self.input_kind != InputKind::MuseBle || self.job_busy {
            return;
        }
        stop_muse_if_any(&self.muse_live);
        {
            let mut st = self.state.lock().unwrap();
            st.last_error = None;
            st.muse_ble_retry_pending = false;
        }
        let h = muse_ble::spawn_muse_ble_thread(Arc::clone(&self.state), self.max_points);
        *self.muse_live.lock().unwrap() = Some(h);
    }

    fn start_record_only(&mut self) {
        if self.job_busy {
            return;
        }
        let state = Arc::clone(&self.state);
        let muse_live = Arc::clone(&self.muse_live);
        let max_points = self.max_points;
        let input_kind = self.input_kind;
        let bin = self.muse_record_bin.clone();
        let mne_script = self.zuna_dir.join("muse_eeg_to_fif.py");
        let out = capture_fif_path(&self.zuna_dir);
        let secs = self.record_secs.max(1);
        let (tx, rx) = mpsc::channel();
        self.job_rx = Some(rx);
        self.job_busy = true;
        self.job_started_at = Some(Instant::now());
        self.job_last_duration = None;
        self.pipeline_step = None;
        self.pipeline_label.clear();
        self.resume_muse_pending = false;
        self.job_log = format!("Recording {secs}s → {} …", out.display());
        thread::spawn(move || {
            if input_kind == InputKind::MuseBle {
                stop_muse_if_any(&muse_live);
            }
            match run_muse_record(&bin, &out, secs as u64, &mne_script) {
                Ok(s) => {
                    let _ = tx.send(JobEvent::LogLine(format!("--- Recording ---\n{s}")));
                    if input_kind == InputKind::MuseBle {
                        let h = muse_ble::spawn_muse_ble_thread(Arc::clone(&state), max_points);
                        *muse_live.lock().unwrap() = Some(h);
                    }
                    let _ = tx.send(JobEvent::Done {
                        ok: true,
                        pipeline_loaded: false,
                        preprocess: false,
                    });
                }
                Err(e) => {
                    let _ = tx.send(JobEvent::LogLine(format!("Recording failed:\n{e}")));
                    if input_kind == InputKind::MuseBle {
                        let h = muse_ble::spawn_muse_ble_thread(Arc::clone(&state), max_points);
                        *muse_live.lock().unwrap() = Some(h);
                    }
                    let _ = tx.send(JobEvent::Done {
                        ok: false,
                        pipeline_loaded: false,
                        preprocess: false,
                    });
                }
            }
        });
    }

    fn start_preprocess(&mut self) {
        if self.job_busy {
            return;
        }
        let state = Arc::clone(&self.state);
        let muse_live = Arc::clone(&self.muse_live);
        let max_points = self.max_points;
        let input_kind = self.input_kind;
        let bin = self.muse_record_bin.clone();
        let zuna_dir = self.zuna_dir.clone();
        let mne_script = self.zuna_dir.join("muse_eeg_to_fif.py");
        let out = capture_fif_path(&self.zuna_dir);
        let basename = out
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_string();
        let secs = self.record_secs.max(1);
        let preset = self.zuna_preset.as_cli();
        let (tx, rx) = mpsc::channel();
        self.job_rx = Some(rx);
        self.job_busy = true;
        self.job_started_at = Some(Instant::now());
        self.job_last_duration = None;
        self.pipeline_step = None;
        self.pipeline_label.clear();
        self.resume_muse_pending = false;
        self.job_log = format!("Recording {secs}s, then ZUNA pipeline (--preset {preset})…\n");
        thread::spawn(move || {
            if input_kind == InputKind::MuseBle {
                stop_muse_if_any(&muse_live);
            }
            match run_muse_record(&bin, &out, secs as u64, &mne_script) {
                Ok(s) => {
                    let _ = tx.send(JobEvent::LogLine(format!("--- Recording ---\n{s}\n")));
                }
                Err(e) => {
                    let _ = tx.send(JobEvent::LogLine(format!("Recording failed:\n{e}")));
                    if input_kind == InputKind::MuseBle {
                        let h = muse_ble::spawn_muse_ble_thread(Arc::clone(&state), max_points);
                        *muse_live.lock().unwrap() = Some(h);
                    }
                    let _ = tx.send(JobEvent::Done {
                        ok: false,
                        pipeline_loaded: false,
                        preprocess: true,
                    });
                    return;
                }
            }
            let input_str = out.to_string_lossy().into_owned();
            match run_zuna_pipeline_streaming(&zuna_dir, &input_str, preset, &tx) {
                Ok(pipeline_ok) => {
                    let mut loaded = false;
                    if pipeline_ok {
                        match load_pipeline_fif_pair(&zuna_dir, &basename, &state, max_points) {
                            Ok(()) => {
                                let err = state.lock().unwrap().last_error.clone();
                                if err.is_some() {
                                    let _ = tx.send(JobEvent::LogLine(format!(
                                        "--- Load pipeline ---\n{}\n",
                                        err.unwrap_or_default()
                                    )));
                                } else {
                                    loaded = true;
                                }
                            }
                            Err(e) => {
                                let _ = tx.send(JobEvent::LogLine(format!(
                                    "--- Load pipeline ---\n{e}\n"
                                )));
                            }
                        }
                    }
                    let need_respawn =
                        !(pipeline_ok && loaded && input_kind == InputKind::MuseBle);
                    if need_respawn && input_kind == InputKind::MuseBle {
                        let h = muse_ble::spawn_muse_ble_thread(Arc::clone(&state), max_points);
                        *muse_live.lock().unwrap() = Some(h);
                    }
                    let _ = tx.send(JobEvent::Done {
                        ok: pipeline_ok,
                        pipeline_loaded: loaded,
                        preprocess: true,
                    });
                }
                Err(e) => {
                    let _ = tx.send(JobEvent::LogLine(format!("--- ZUNA pipeline ---\n{e}\n")));
                    if input_kind == InputKind::MuseBle {
                        let h = muse_ble::spawn_muse_ble_thread(Arc::clone(&state), max_points);
                        *muse_live.lock().unwrap() = Some(h);
                    }
                    let _ = tx.send(JobEvent::Done {
                        ok: false,
                        pipeline_loaded: false,
                        preprocess: true,
                    });
                }
            }
        });
    }
}

/// Derive toolbar status from job log content.
fn muse_status_from_job_log(log: &str, success: bool) -> String {
    if !success {
        if log.contains("Timed out scanning") {
            return "Muse: not connected (Bluetooth scan timed out)".to_string();
        }
        if log.contains("MNE script not found") {
            return "Muse: script missing (check --zuna-dir)".to_string();
        }
        return "Muse: last job failed — see log below".to_string();
    }
    if log.contains("--- ZUNA pipeline ---") && log.contains("Done.") {
        return "Muse: OK — recording + ZUNA pipeline finished".to_string();
    }
    if log.contains("--- Recording ---") {
        return "Muse: OK — recording saved".to_string();
    }
    "Muse: last job finished".to_string()
}

/// Writes under `zuna_dir/live_captures/live_<timestamp_ms>.fif`.
fn capture_fif_path(zuna_dir: &Path) -> PathBuf {
    let dir = zuna_dir.join("live_captures");
    let _ = std::fs::create_dir_all(&dir);
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0);
    dir.join(format!("live_{ts}.fif"))
}

fn run_muse_record(
    bin: &Path,
    output: &Path,
    duration_secs: u64,
    mne_script: &Path,
) -> Result<String, String> {
    if !bin.is_file() {
        return Err(format!(
            "muse-record-fif not found at {}.\nBuild: cd app && cargo build --release --bin muse-record-fif",
            bin.display()
        ));
    }
    if !mne_script.is_file() {
        return Err(format!(
            "MNE script not found: {}.\nExpected next to ZUNA (--zuna-dir); repo path is services/zuna/muse_eeg_to_fif.py",
            mne_script.display()
        ));
    }
    let out = Command::new(bin)
        .arg("-o")
        .arg(output)
        .arg("-d")
        .arg(duration_secs.to_string())
        .arg("--script")
        .arg(mne_script)
        .output()
        .map_err(|e| format!("spawn {}: {e}", bin.display()))?;

    let mut combined = String::new();
    let stdout = String::from_utf8_lossy(&out.stdout);
    let stderr = String::from_utf8_lossy(&out.stderr);
    if !stdout.is_empty() {
        combined.push_str("--- stdout ---\n");
        combined.push_str(&stdout);
    }
    if !stderr.is_empty() {
        combined.push_str("--- stderr ---\n");
        combined.push_str(&stderr);
    }
    if combined.is_empty() {
        combined.push_str("(no output)");
    }

    if out.status.success() {
        Ok(combined)
    } else {
        Err(combined)
    }
}

/// Parse `[n/t] label` step lines from `run_fif_pipeline.py` stdout.
fn parse_step_line(line: &str) -> Option<(u8, u8, String)> {
    let line = line.trim();
    let rest = line.strip_prefix('[')?;
    let (step_part, after_bracket) = rest.split_once(']')?;
    let (n_str, t_str) = step_part.split_once('/')?;
    let n: u8 = n_str.parse().ok()?;
    let t: u8 = t_str.parse().ok()?;
    let label = after_bracket.trim().to_string();
    Some((n, t, label))
}

fn run_zuna_pipeline_streaming(
    zuna_dir: &Path,
    input: &str,
    preset: &str,
    tx: &mpsc::Sender<JobEvent>,
) -> Result<bool, String> {
    let mut cmd = Command::new("uv");
    cmd.args(["run", "python", "run_fif_pipeline.py"]);
    cmd.current_dir(zuna_dir)
        .arg("--input")
        .arg(input)
        .arg("--work-dir")
        .arg(ZUNA_PIPELINE_WORK_DIR)
        .arg("--preset")
        .arg(preset)
        .arg("--headless")
        .env("MPLBACKEND", "Agg")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    let mut child = cmd.spawn().map_err(|e| {
        format!(
            "failed to spawn pipeline in {}: {e}\n(Is `uv` on PATH?)",
            zuna_dir.display()
        )
    })?;

    let stdout = child.stdout.take().unwrap();
    let stderr = child.stderr.take().unwrap();
    let txo = tx.clone();
    let h1 = thread::spawn(move || {
        for line in BufReader::new(stdout).lines() {
            let line = line.unwrap_or_default();
            let _ = txo.send(JobEvent::LogLine(line.clone() + "\n"));
            if let Some((n, t, label)) = parse_step_line(&line) {
                let _ = txo.send(JobEvent::Step { n, total: t, label });
            }
        }
    });
    let txe = tx.clone();
    let h2 = thread::spawn(move || {
        for line in BufReader::new(stderr).lines() {
            let line = line.unwrap_or_default();
            let _ = txe.send(JobEvent::LogLine(format!("{line}\n")));
        }
    });
    h1.join().map_err(|_| "stdout reader panic".to_string())?;
    h2.join().map_err(|_| "stderr reader panic".to_string())?;
    let status = child.wait().map_err(|e| format!("wait: {e}"))?;
    Ok(status.success())
}

fn clear_plot_buffers(state: &mut SharedState) {
    state.channels_orig.clear();
    state.channels_recon.clear();
    state.frames_received = 0;
    state.channel_names = None;
    state.last_error = None;
}

fn is_fif_extension(path: &Path) -> bool {
    path.extension()
        .and_then(|s| s.to_str())
        .map(|e| e.eq_ignore_ascii_case("fif") || e.eq_ignore_ascii_case("fiff"))
        .unwrap_or(false)
}

/// Match a FIF pair across `1_fif_filter` and `4_fif_output`, preferring the capture basename.
fn resolve_pipeline_fif_pair(work: &Path, capture_basename: &str) -> Result<(PathBuf, PathBuf), String> {
    let filter_dir = work.join("1_fif_filter");
    let out_dir = work.join("4_fif_output");
    if !filter_dir.is_dir() {
        return Err(format!("missing directory: {}", filter_dir.display()));
    }
    if !out_dir.is_dir() {
        return Err(format!("missing directory: {}", out_dir.display()));
    }

    let preferred_o = filter_dir.join(capture_basename);
    let preferred_r = out_dir.join(capture_basename);
    if preferred_o.is_file() && preferred_r.is_file() {
        return Ok((preferred_o, preferred_r));
    }

    let mut pairs: Vec<(PathBuf, PathBuf, std::time::SystemTime)> = Vec::new();
    let rd = fs::read_dir(&filter_dir).map_err(|e| format!("read {}: {e}", filter_dir.display()))?;
    for ent in rd.filter_map(|e| e.ok()) {
        let p = ent.path();
        if !is_fif_extension(&p) {
            continue;
        }
        let name = match p.file_name() {
            Some(n) => n.to_owned(),
            None => continue,
        };
        let recon = out_dir.join(&name);
        if recon.is_file() {
            let mt = fs::metadata(&p)
                .and_then(|m| m.modified())
                .unwrap_or(std::time::UNIX_EPOCH);
            pairs.push((p, recon, mt));
        }
    }
    if pairs.is_empty() {
        return Err(format!(
            "no matching .fif in {} and {} (tried {}). ZUNA may have failed before writing outputs.",
            filter_dir.display(),
            out_dir.display(),
            capture_basename
        ));
    }
    if let Some((o, r, _)) = pairs.iter().find(|(o, _, _)| {
        o.file_name()
            .and_then(|s| s.to_str())
            .map(|s| s == capture_basename)
            .unwrap_or(false)
    }) {
        return Ok((o.clone(), r.clone()));
    }
    pairs.sort_by(|a, b| b.2.cmp(&a.2));
    Ok((pairs[0].0.clone(), pairs[0].1.clone()))
}

fn load_pipeline_fif_pair(
    zuna_dir: &Path,
    capture_basename: &str,
    state: &Arc<Mutex<SharedState>>,
    max_points: usize,
) -> Result<(), String> {
    let work = zuna_dir.join(ZUNA_PIPELINE_WORK_DIR);
    let (orig_path, recon_path) = resolve_pipeline_fif_pair(&work, capture_basename)?;

    let mut cmd = Command::new("uv");
    cmd.args(["run", "python", "fif_pair_to_eegd.py"]);
    cmd.current_dir(zuna_dir)
        .arg("--original")
        .arg(&orig_path)
        .arg("--reconstructed")
        .arg(&recon_path)
        .arg("--max-points")
        .arg(max_points.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());

    let mut child = cmd
        .spawn()
        .map_err(|e| format!("spawn fif_pair_to_eegd.py: {e}"))?;

    let stdout = child.stdout.take().unwrap();
    let mut stderr = child.stderr.take().unwrap();
    let err_h = thread::spawn(move || {
        let mut s = String::new();
        let _ = stderr.read_to_string(&mut s);
        s
    });

    let mut reader = BufReader::new(stdout);
    let mut line = String::new();
    let mut names: Option<Vec<String>> = None;
    let mut found_names = false;
    for _ in 0..512 {
        line.clear();
        let n = reader
            .read_line(&mut line)
            .map_err(|e| format!("read NAMES line: {e}"))?;
        if n == 0 {
            return Err("EOF before NAMES: line from fif_pair_to_eegd.py".to_string());
        }
        let line_trim = line.trim_end();
        if let Some(rest) = line_trim.strip_prefix("NAMES:") {
            names = if rest.is_empty() {
                None
            } else {
                Some(rest.split(',').map(|s| s.to_string()).collect())
            };
            found_names = true;
            break;
        }
    }
    if !found_names {
        return Err(
            "no NAMES: line in fif_pair_to_eegd.py stdout (unexpected preamble after 512 lines)"
                .to_string(),
        );
    }

    {
        let mut st = state.lock().unwrap();
        clear_plot_buffers(&mut st);
    }

    let mut raw = Vec::new();
    read_frames_from(&mut reader, &mut raw, state, max_points)
        .map_err(|e| format!("read EEGD frames: {e}"))?;

    let err_s = err_h.join().map_err(|_| "stderr reader panic".to_string())?;
    let status = child.wait().map_err(|e| format!("wait: {e}"))?;
    if !status.success() {
        return Err(format!("fif_pair_to_eegd.py failed: {err_s}"));
    }

    {
        let mut st = state.lock().unwrap();
        if let Some(e) = st.last_error.take() {
            clear_plot_buffers(&mut st);
            return Err(e);
        }
        if st.channels_orig.is_empty() {
            clear_plot_buffers(&mut st);
            return Err(
                "pipeline load produced no EEG samples (check FIF overlap and length)".to_string(),
            );
        }
        st.channel_names = names;
    }
    Ok(())
}

fn format_duration(d: Duration) -> String {
    let secs = d.as_secs_f64();
    if secs >= 3600.0 {
        let h = (secs / 3600.0).floor() as u64;
        let m = ((secs % 3600.0) / 60.0).floor() as u64;
        let s = (secs % 60.0).round() as u64;
        format!("{h}h {m}m {s}s")
    } else if secs >= 60.0 {
        let m = (secs / 60.0).floor() as u64;
        let s = (secs % 60.0).round() as u64;
        format!("{m}m {s}s")
    } else {
        format!("{secs:.1}s")
    }
}

fn channel_label(names: Option<&[String]>, n_ch: usize, i: usize) -> String {
    if let Some(ns) = names {
        if i < ns.len() {
            return ns[i].clone();
        }
    }
    if n_ch == CHANNEL_NAMES_MUSE_4.len() && i < CHANNEL_NAMES_MUSE_4.len() {
        CHANNEL_NAMES_MUSE_4[i].to_string()
    } else {
        format!("ch{i}")
    }
}

impl eframe::App for EegViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();
        self.poll_job();

        let (
            n_ch,
            frames,
            has_recon_stream,
            last_error,
            tcp_connected,
            muse_ble_streaming,
            muse_ble_retry_pending,
        ) = {
            let st = self.state.lock().unwrap();
            let has = !st.channels_recon.is_empty()
                && st.channels_recon.len() == st.channels_orig.len();
            (
                st.channels_orig.len(),
                st.frames_received,
                has,
                st.last_error.clone(),
                st.tcp_client_connected,
                st.muse_ble_streaming,
                st.muse_ble_retry_pending,
            )
        };

        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.show_live, "Show live").on_hover_text(
                    "When off, live streaming (Muse / stdin / TCP) is hidden. A loaded ZUNA comparison (original vs reconstructed) still appears.",
                );
                ui.separator();
                ui.label("Duration (s):");
                ui.add(
                    egui::DragValue::new(&mut self.record_secs)
                        .range(1..=3600)
                        .speed(1.0),
                );
                ui.separator();
                ui.label("ZUNA preset:");
                if ui
                    .add_enabled(!self.job_busy, egui::Button::new("Record"))
                    .on_hover_text(
                        "Record from Muse → services/zuna/live_captures/live_<time>.fif (needs muse-record-fif)",
                    )
                    .clicked()
                {
                    self.start_record_only();
                }
                ui.separator();
                egui::ComboBox::from_id_salt("zuna_preset")
                    .selected_text(self.zuna_preset.label())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.zuna_preset, ZunaPreset::None, ZunaPreset::None.label());
                        ui.selectable_value(&mut self.zuna_preset, ZunaPreset::Ch8, ZunaPreset::Ch8.label());
                        ui.selectable_value(
                            &mut self.zuna_preset,
                            ZunaPreset::Ch10,
                            ZunaPreset::Ch10.label(),
                        );
                    });
                if ui
                    .add_enabled(!self.job_busy, egui::Button::new("Preprocess"))
                    .on_hover_text(
                        "Record for the duration above, then run ZUNA run_fif_pipeline.py (preset from menu)",
                    )
                    .clicked()
                {
                    self.start_preprocess();
                }
                if self.job_busy {
                    ui.spinner();
                }
            });
            ui.horizontal(|ui| {
                ui.label("Plot source:");
                match self.input_kind {
                    InputKind::MuseBle => {
                        if muse_ble_streaming {
                            ui.colored_label(egui::Color32::from_rgb(80, 200, 120), "● Muse BLE");
                        } else if muse_ble_retry_pending {
                            ui.label(RichText::new("○ Muse BLE (timed out)").weak());
                        } else {
                            ui.label(RichText::new("○ Muse BLE (connecting…)").weak());
                        }
                    }
                    InputKind::Stdin => {
                        ui.label("stdin — pipe EEGF/EEGD");
                    }
                    InputKind::Tcp => {
                        if tcp_connected {
                            ui.colored_label(egui::Color32::from_rgb(80, 200, 120), "● TCP client");
                        } else {
                            ui.label(RichText::new("○ TCP listening…").weak());
                        }
                    }
                    InputKind::None => {
                        ui.label(RichText::new("off (--no-muse-ble)").weak());
                    }
                }
                ui.separator();
                ui.label("Batch:");
                if self.job_busy {
                    ui.spinner();
                    ui.label("recording / ZUNA…");
                } else {
                    ui.label(RichText::new(&self.muse_status).weak()).on_hover_text(
                        "Record / Preprocess: Bluetooth Muse → FIF → ZUNA. Live plot follows Plot source.",
                    );
                }
                if self.resume_muse_pending {
                    if ui
                        .button("Resume Muse")
                        .on_hover_text(
                            "Pipeline comparison is loaded; resume Bluetooth live stream when ready.",
                        )
                        .clicked()
                    {
                        self.resume_muse_ble();
                    }
                }
            });
            if self.job_busy
                || self.pipeline_step.is_some()
                || self.job_last_duration.is_some()
            {
                ui.horizontal(|ui| {
                    if let Some((n, t)) = self.pipeline_step {
                        ui.label(format!("Step {n}/{t}"));
                        if !self.pipeline_label.is_empty() {
                            ui.label(RichText::new(&self.pipeline_label).weak());
                        }
                    }
                    if self.job_busy {
                        if let Some(t0) = self.job_started_at {
                            ui.label(format!(
                                "elapsed: {}",
                                format_duration(t0.elapsed())
                            ));
                        }
                    } else if let Some(d) = self.job_last_duration {
                        ui.label(format!("last run: {}", format_duration(d)));
                    }
                });
            }
            if n_ch > 0 || frames > 0 {
                ui.horizontal(|ui| {
                    ui.label(format!("channels: {n_ch}"));
                    ui.separator();
                    ui.label(format!("frames: {frames}"));
                });
            }
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.show_original, "Original (blue)");
                ui.add_enabled_ui(has_recon_stream, |ui| {
                    ui.checkbox(&mut self.show_reconstructed, "Reconstructed (red)");
                });
                if !has_recon_stream {
                    ui.label(RichText::new("(red trace when EEGD stream)").weak().small());
                }
            });
            if let Some(ref e) = last_error {
                ui.horizontal(|ui| {
                    ui.colored_label(egui::Color32::RED, e);
                    if self.input_kind == InputKind::MuseBle
                        && muse_ble_retry_pending
                        && !self.job_busy
                    {
                        if ui
                            .button("Retry Muse")
                            .on_hover_text("Scan and connect again (after a timeout or handshake failure).")
                            .clicked()
                        {
                            self.retry_muse_ble();
                        }
                    }
                });
            }
            if !self.job_log.is_empty() {
                egui::ScrollArea::vertical()
                    .max_height(160.0)
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new(&self.job_log)
                                .monospace()
                                .size(11.0),
                        );
                    });
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let show_comparison =
                n_ch > 0 && (has_recon_stream || self.resume_muse_pending);
            let show_plot = self.show_live || show_comparison;

            if !show_plot {
                ui.vertical_centered(|ui| {
                    ui.label(
                        "Live view is off. Enable “Show live” for streaming, or run Preprocess to load a comparison here.",
                    );
                });
                return;
            }

            if n_ch == 0 {
                let hint = if self.resume_muse_pending {
                    "ZUNA comparison loaded — use “Resume Muse” in the toolbar when you want live Bluetooth again."
                } else {
                    match self.input_kind {
                        InputKind::MuseBle => {
                            if muse_ble_retry_pending {
                                "No EEG yet — use Retry Muse next to the error in the toolbar."
                            } else if muse_ble_streaming {
                                "Receiving Muse EEG…"
                            } else {
                                "Connecting to Muse over Bluetooth…"
                            }
                        }
                        InputKind::Stdin => "Waiting for EEGF/EEGD on stdin…",
                        InputKind::Tcp => "Waiting for a TCP client (EEGF/EEGD)…",
                        InputKind::None => {
                            "No plot input. Run without --no-muse-ble, or use --stdin / --tcp."
                        }
                    }
                };
                ui.vertical_centered(|ui| {
                    ui.label(hint);
                });
                return;
            }

            let st = self.state.lock().unwrap();
            let has_recon = !st.channels_recon.is_empty()
                && st.channels_recon.len() == st.channels_orig.len();

            let show_orig = self.show_original;
            let show_recon = self.show_reconstructed;

            let plot_h = (ui.available_height() / n_ch as f32).clamp(72.0, 220.0);

            // One stacked plot per channel.
            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    let names = st.channel_names.as_deref();
                    for i in 0..n_ch {
                        let base = channel_label(names, n_ch, i);

                        ui.horizontal(|ui| {
                            ui.allocate_ui_with_layout(
                                egui::vec2(52.0, plot_h),
                                egui::Layout::top_down(egui::Align::Center),
                                |ui| {
                                    ui.add_space(plot_h * 0.35);
                                    ui.label(
                                        RichText::new(&base)
                                            .strong()
                                            .monospace()
                                            .size(13.0),
                                    );
                                },
                            );

                            Plot::new(format!("eeg_ch_{i}"))
                                .height(plot_h)
                                .width(ui.available_width())
                                // X = sample index: zoom/pan/scroll OK. Y = amplitude: off so vertical
                                // wheel/drag doesn’t fight the outer ScrollArea or rescale µV by accident.
                                .allow_zoom(egui::Vec2b::new(true, false))
                                .allow_drag(egui::Vec2b::new(true, false))
                                .allow_scroll(egui::Vec2b::new(true, false))
                                .show_axes(true)
                                .show_grid(true)
                                .legend(
                                    Legend::default().position(egui_plot::Corner::RightTop),
                                )
                                .show(ui, |plot_ui| {
                                    if show_orig {
                                        let ch = &st.channels_orig[i];
                                        let n = ch.len();
                                        let pts: PlotPoints = (0..n)
                                            .map(|t| [t as f64, ch[t] as f64])
                                            .collect();
                                        plot_ui.line(
                                            Line::new(pts)
                                                .name("Original")
                                                .color(COLOR_ORIGINAL),
                                        );
                                    }

                                    if has_recon && show_recon {
                                        let ch = &st.channels_recon[i];
                                        let n = ch.len();
                                        let pts: PlotPoints = (0..n)
                                            .map(|t| [t as f64, ch[t] as f64])
                                            .collect();
                                        plot_ui.line(
                                            Line::new(pts)
                                                .name("Reconstructed")
                                                .color(COLOR_RECONSTRUCTED),
                                        );
                                    }
                                });
                        });

                        ui.add_space(4.0);
                    }

                    if n_ch > 0 {
                        ui.label(RichText::new("Sample index →").small().weak());
                    }
                });
        });
    }
}

fn read_u32<R: Read>(r: &mut R) -> io::Result<u32> {
    let mut b = [0u8; 4];
    r.read_exact(&mut b)?;
    Ok(u32::from_le_bytes(b))
}

enum FrameKind {
    Single(Vec<Vec<f32>>),
    Dual {
        orig: Vec<Vec<f32>>,
        recon: Vec<Vec<f32>>,
    },
}

fn floats_to_channel_major(
    out: Vec<f32>,
    n_channels: u32,
    n_samples: u32,
) -> io::Result<Vec<Vec<f32>>> {
    let total = (n_channels as usize) * (n_samples as usize);
    if out.len() != total {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "float buffer length mismatch",
        ));
    }
    let mut by_channel = Vec::with_capacity(n_channels as usize);
    for c in 0..n_channels as usize {
        let start = c * n_samples as usize;
        let end = start + n_samples as usize;
        by_channel.push(out[start..end].to_vec());
    }
    Ok(by_channel)
}

fn read_frame<R: Read>(r: &mut R, buf: &mut Vec<u8>) -> io::Result<Option<FrameKind>> {
    let magic = match read_u32(r) {
        Ok(m) => m,
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e),
    };

    let n_channels = read_u32(r)?;
    let n_samples = read_u32(r)?;
    if n_channels == 0 || n_samples == 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "n_channels and n_samples must be > 0",
        ));
    }

    let total = (n_channels as usize)
        .checked_mul(n_samples as usize)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "size overflow"))?;
    let nbytes = total
        .checked_mul(4)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "size overflow"))?;

    match magic {
        MAGIC_SINGLE => {
            buf.resize(nbytes, 0);
            r.read_exact(buf)?;
            let mut out = Vec::with_capacity(total);
            for chunk in buf.chunks_exact(4) {
                out.push(f32::from_le_bytes(chunk.try_into().unwrap()));
            }
            let by_channel = floats_to_channel_major(out, n_channels, n_samples)?;
            Ok(Some(FrameKind::Single(by_channel)))
        }
        MAGIC_DUAL => {
            let nbytes_dual = nbytes
                .checked_mul(2)
                .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "size overflow"))?;
            buf.resize(nbytes_dual, 0);
            r.read_exact(buf)?;
            let mut out = Vec::with_capacity(total * 2);
            for chunk in buf.chunks_exact(4) {
                out.push(f32::from_le_bytes(chunk.try_into().unwrap()));
            }
            let orig_f = out[..total].to_vec();
            let recon_f = out[total..].to_vec();
            let orig = floats_to_channel_major(orig_f, n_channels, n_samples)?;
            let recon = floats_to_channel_major(recon_f, n_channels, n_samples)?;
            Ok(Some(FrameKind::Dual { orig, recon }))
        }
        _ => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("bad magic {magic:#x}, expected {MAGIC_SINGLE:#x} or {MAGIC_DUAL:#x}"),
        )),
    }
}

fn trim_front(v: &mut Vec<f32>, max_points: usize) {
    if v.len() > max_points {
        let drop = v.len() - max_points;
        v.drain(0..drop);
    }
}

fn merge_stack(
    acc: &mut Vec<Vec<f32>>,
    incoming: Vec<Vec<f32>>,
    max_points: usize,
) {
    if acc.is_empty() {
        let mut by_channel = incoming;
        for v in &mut by_channel {
            trim_front(v, max_points);
        }
        *acc = by_channel;
    } else if acc.len() != incoming.len() {
        let mut by_channel = incoming;
        for v in &mut by_channel {
            trim_front(v, max_points);
        }
        *acc = by_channel;
    } else {
        for (a, chunk) in acc.iter_mut().zip(incoming.iter()) {
            a.extend_from_slice(chunk);
            trim_front(a, max_points);
        }
    }
}

pub(crate) fn merge_single(state: &mut SharedState, by_channel: Vec<Vec<f32>>, max_points: usize) {
    state.channels_recon.clear();
    state.channel_names = None;
    merge_stack(&mut state.channels_orig, by_channel, max_points);
    state.frames_received = state.frames_received.saturating_add(1);
}

fn merge_dual(
    state: &mut SharedState,
    orig: Vec<Vec<f32>>,
    recon: Vec<Vec<f32>>,
    max_points: usize,
) {
    if orig.len() != recon.len() {
        state.last_error = Some(format!(
            "dual frame: orig ch {} != recon ch {}",
            orig.len(),
            recon.len()
        ));
        return;
    }
    if state.channels_orig.is_empty() {
        state.channel_names = None;
    }
    state.last_error = None;
    merge_stack(&mut state.channels_orig, orig, max_points);
    merge_stack(&mut state.channels_recon, recon, max_points);
    state.frames_received = state.frames_received.saturating_add(1);
}

fn read_frames_from<R: Read>(
    r: &mut R,
    raw: &mut Vec<u8>,
    state: &Arc<Mutex<SharedState>>,
    max_points: usize,
) -> io::Result<()> {
    loop {
        match read_frame(r, raw) {
            Ok(Some(FrameKind::Single(by_ch))) => {
                let mut st = state.lock().unwrap();
                st.last_error = None;
                merge_single(&mut st, by_ch, max_points);
            }
            Ok(Some(FrameKind::Dual { orig, recon })) => {
                let mut st = state.lock().unwrap();
                merge_dual(&mut st, orig, recon, max_points);
            }
            Ok(None) => break,
            Err(e) => {
                let mut st = state.lock().unwrap();
                st.last_error = Some(format!("read error: {e}"));
                break;
            }
        }
    }
    Ok(())
}

fn main() -> eframe::Result<()> {
    let args = Args::parse();
    let input_kind = resolve_input_kind(&args);
    let state = Arc::new(Mutex::new(SharedState::default()));
    let max_points = args.max_points;
    let muse_live = Arc::new(Mutex::new(None));

    match input_kind {
        InputKind::Tcp => {
            let addr = args.tcp.as_ref().expect("tcp");
            let sa: SocketAddr = addr.parse().unwrap_or_else(|e| {
                eprintln!("bad --tcp address: {e}");
                std::process::exit(1);
            });
            let state_tcp = Arc::clone(&state);
            thread::spawn(move || {
                let listener = match TcpListener::bind(sa) {
                    Ok(l) => l,
                    Err(e) => {
                        state_tcp.lock().unwrap().last_error = Some(format!("bind: {e}"));
                        return;
                    }
                };
                if let Ok((mut stream, _)) = listener.accept() {
                    state_tcp.lock().unwrap().tcp_client_connected = true;
                    let _ = stream.set_nodelay(true);
                    let mut raw = Vec::new();
                    if let Err(e) = read_frames_from(&mut stream, &mut raw, &state_tcp, max_points) {
                        state_tcp.lock().unwrap().last_error = Some(format!("tcp read: {e}"));
                    }
                }
            });
        }
        InputKind::Stdin => {
            let state_in = Arc::clone(&state);
            thread::spawn(move || {
                let stdin = io::stdin();
                let mut locked = stdin.lock();
                let mut raw = Vec::new();
                let _ = read_frames_from(&mut locked, &mut raw, &state_in, max_points);
            });
        }
        InputKind::MuseBle => {
            let h = muse_ble::spawn_muse_ble_thread(Arc::clone(&state), max_points);
            *muse_live.lock().unwrap() = Some(h);
        }
        InputKind::None => {}
    }

    let muse_status = match input_kind {
        InputKind::MuseBle => "Muse BLE: connecting…".to_string(),
        InputKind::Stdin => "Batch: idle (plot from stdin)".to_string(),
        InputKind::Tcp => "Batch: idle (plot from TCP)".to_string(),
        InputKind::None => "Batch: idle".to_string(),
    };

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("eeg-viewer — Muse / ZUNA")
            .with_inner_size([920.0, 680.0]),
        ..Default::default()
    };

    let zuna_dir = args.zuna_dir.clone();
    let muse_record_bin = args.muse_record_bin.clone();
    let muse_live_app = Arc::clone(&muse_live);

    eframe::run_native(
        "eeg-viewer",
        options,
        Box::new(move |_cc| {
            Ok(Box::new(EegViewerApp {
                state,
                show_live: true,
                show_original: true,
                show_reconstructed: true,
                record_secs: 120,
                zuna_dir,
                muse_record_bin,
                job_busy: false,
                job_log: String::new(),
                job_rx: None,
                muse_status,
                zuna_preset: ZunaPreset::default(),
                muse_live: muse_live_app,
                max_points,
                input_kind,
                job_started_at: None,
                job_last_duration: None,
                pipeline_step: None,
                pipeline_label: String::new(),
                resume_muse_pending: false,
            }) as Box<dyn eframe::App>)
        }),
    )
}
