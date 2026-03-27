//! Multi-Muse EEG viewer with per-device recording and inference server connection.
//!
//! Supports:
//! - Up to 4 Muse headsets via BLE (scan, pick, connect)
//! - Per-device tabbed display with independent recording
//! - Inference server connection (EEGM protocol) for real-time ZUNA processing
//! - Legacy EEGF/EEGD stdin/TCP pipe mode (single-device)

mod device_state;
mod muse_ble;
mod signal_pipeline;
mod tcp_client;
mod viz_aura;
mod viz_topo;

use std::collections::HashSet;
use std::fs;
use std::io::{self, BufRead, BufReader, Read};
use std::net::{SocketAddr, TcpListener};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::Ordering;
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use eframe::egui;
use egui::RichText;
use egui_plot::{Legend, Line, Plot, PlotPoints};

use device_state::{
    clamp_tab_index, device_fif_path, next_available_headband_id, session_file_path,
    ConnectedDevice, DeviceState, ServerState, SharedOutboundTx,
};
use muse_ble::ScanResult;
use signal_pipeline::{
    AnalysisFrame, BaselineState, DimensionReading, PipelineCommand, VizMode,
};
use tcp_client::DeviceMap;

const MAGIC_SINGLE: u32 = 0x4545_4746; // "EEGF"
const MAGIC_DUAL: u32 = 0x4545_4744; // "EEGD"

const ZUNA_PIPELINE_WORK_DIR: &str = "zuna_pipeline_run";
const CHANNEL_NAMES_MUSE_4: [&str; 4] = ["TP9", "AF7", "AF8", "TP10"];
const SAMPLE_RATE_HZ: u32 = 256;

#[derive(Clone, Copy, PartialEq, Eq, Default)]
#[allow(dead_code)]
enum ZunaPreset {
    #[default]
    None,
    Ch8,
    Ch10,
}

/// Plot input source (legacy single-device modes).
#[derive(Clone, Copy, PartialEq, Eq)]
enum InputKind {
    MuseBle,
    Stdin,
    Tcp,
    None,
}

#[allow(dead_code)]
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
            ZunaPreset::Ch8 => "8ch 10-20",
            ZunaPreset::Ch10 => "10ch 10-20",
        }
    }
}

#[derive(Parser, Debug)]
#[command(name = "eeg-viewer")]
struct Args {
    #[arg(long, default_value_t = 8192)]
    max_points: usize,

    /// Read EEGF/EEGD from stdin (legacy single-device mode).
    #[arg(long)]
    stdin: bool,

    /// Listen for EEGF/EEGD frames on a TCP socket (legacy single-device mode).
    #[arg(long)]
    tcp: Option<String>,

    /// Disable Muse Bluetooth (legacy flag).
    #[arg(long)]
    no_muse_ble: bool,

    /// Inference server address (e.g. 127.0.0.1:9100).
    #[arg(long)]
    server: Option<String>,

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

    /// Launch with a simulated Muse device (synthetic EEG, no BLE needed).
    #[arg(long)]
    simulate: bool,
}

/// Job events from background Record / Preprocess threads.
enum JobEvent {
    LogLine(String),
    Step { n: u8, total: u8, label: String },
    Done { ok: bool },
}

// ── Legacy SharedState for stdin/TCP pipe modes ─────────────────────────────

#[derive(Default)]
struct SharedState {
    channels_orig: Vec<Vec<f32>>,
    channels_recon: Vec<Vec<f32>>,
    frames_received: u64,
    last_error: Option<String>,
    tcp_client_connected: bool,
    channel_names: Option<Vec<String>>,
    recording_active: bool,
    record_buffer: Vec<Vec<f32>>,
    record_sample_count: usize,
}

struct EegViewerApp {
    // ── Multi-device ──
    connected_devices: Vec<ConnectedDevice>,
    active_tab: usize,

    // ── Scanning ──
    scan_results: Vec<ScanResult>,
    scan_active: bool,
    scan_rx: Option<std::sync::mpsc::Receiver<Result<Vec<ScanResult>, String>>>,
    show_device_picker: bool,

    // ── Inference server ──
    server_state: Arc<Mutex<ServerState>>,
    server_addr: Option<String>,
    outbound_tx: SharedOutboundTx,
    device_map: DeviceMap,

    // ── Display toggles ──
    show_live: bool,
    show_raw: bool,
    show_reconstructed: bool,
    viz_mode: VizMode,

    // ── Recording / ZUNA ──
    zuna_dir: PathBuf,
    #[allow(dead_code)]
    muse_record_bin: PathBuf,
    zuna_preset: ZunaPreset,
    #[allow(dead_code)]
    record_secs: u32,

    // ── Background job ──
    job_busy: bool,
    job_log: String,
    job_rx: Option<Receiver<JobEvent>>,
    job_device_idx: Option<usize>,
    job_started_at: Option<Instant>,
    job_last_duration: Option<Duration>,
    pipeline_step: Option<(u8, u8)>,
    pipeline_label: String,

    // ── Config ──
    max_points: usize,
    input_kind: InputKind,
    tokio_rt: Arc<tokio::runtime::Runtime>,

    // ── Legacy single-device state (stdin/TCP modes) ──
    legacy_state: Option<Arc<Mutex<SharedState>>>,
    muse_status: String,
    #[allow(dead_code)]
    resume_muse_pending: bool,
    #[allow(dead_code)]
    reset_confirm_pending: bool,
}

const COLOR_ORIGINAL: egui::Color32 = egui::Color32::from_rgb(0, 128, 255);
const COLOR_RECONSTRUCTED: egui::Color32 = egui::Color32::from_rgb(255, 48, 48);

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

// ── EegViewerApp: Multi-device methods ──────────────────────────────────────

impl EegViewerApp {
    fn start_scan(&mut self) {
        if self.scan_active {
            return;
        }
        self.scan_active = true;
        self.scan_rx = Some(muse_ble::spawn_scan_task(&self.tokio_rt, 20));
    }

    fn poll_scan(&mut self) {
        let Some(rx) = self.scan_rx.as_ref() else {
            return;
        };
        match rx.try_recv() {
            Ok(Ok(results)) => {
                self.scan_results = results;
                self.scan_active = false;
                self.scan_rx = None;
                self.show_device_picker = true;
            }
            Ok(Err(err)) => {
                self.muse_status = err;
                self.scan_active = false;
                self.scan_rx = None;
            }
            Err(TryRecvError::Empty) => {}
            Err(TryRecvError::Disconnected) => {
                self.scan_active = false;
                self.scan_rx = None;
            }
        }
    }

    fn connect_device(&mut self, scan_idx: usize) {
        if scan_idx >= self.scan_results.len() {
            return;
        }

        let headband_id = match next_available_headband_id(&self.connected_devices) {
            Some(id) => id,
            None => return,
        };

        let scan_result = &self.scan_results[scan_idx];
        if self
            .connected_devices
            .iter()
            .any(|d| d.device_id == scan_result.device.id)
        {
            return;
        }

        let device_name = scan_result.device.name.clone();
        let device_id = scan_result.device.id.clone();
        let display_label = scan_result.display_label.clone();

        let state = Arc::new(Mutex::new(DeviceState::new(
            device_name,
            device_id.clone(),
            self.max_points,
        )));

        let sr = self.scan_results.remove(scan_idx);
        let sess_path = session_file_path(&self.zuna_dir, &display_label);

        let (cmd_tx, cmd_rx) = tokio::sync::mpsc::unbounded_channel();

        let ble_handle = muse_ble::spawn_device_ble_task(
            sr.device,
            headband_id,
            Arc::clone(&state),
            self.outbound_tx.clone(),
            Some(sess_path.clone()),
            cmd_rx,
            &self.tokio_rt,
        );

        // Register in shared device map so TCP recv/send tasks can route by headband_id
        self.device_map
            .lock()
            .unwrap()
            .insert(headband_id.as_u32(), Arc::clone(&state));

        self.connected_devices.push(ConnectedDevice {
            device_name: display_label,
            device_id,
            headband_id,
            state,
            stop_flag: ble_handle.stop,
            join_handle: Some(ble_handle.join),
            last_saved_fif: None,
            record_started_at: None,
            last_record_toggle: None,
            epoch_seq: 0,
            session_path: Some(sess_path),
            pipeline_cmd_tx: Some(cmd_tx),
        });

        self.active_tab = self.connected_devices.len() - 1;
    }

    fn disconnect_device(&mut self, idx: usize) {
        if idx >= self.connected_devices.len() {
            return;
        }
        let device = self.connected_devices.remove(idx);
        device.stop_flag.store(true, Ordering::SeqCst);
        self.device_map
            .lock()
            .unwrap()
            .remove(&device.headband_id.as_u32());
        self.active_tab = clamp_tab_index(self.active_tab, self.connected_devices.len());
    }

    /// Spawn a simulated device that generates synthetic EEG data.
    fn spawn_simulated_device(&mut self) {
        use std::sync::atomic::AtomicBool;
        use device_state::HeadbandId;
        use signal_pipeline::{ChannelId, SignalPipeline};

        let headband_id = HeadbandId::new(0).unwrap();
        let state = Arc::new(Mutex::new(DeviceState::new(
            "Simulated".to_string(),
            "sim-0000".to_string(),
            self.max_points,
        )));
        let stop = Arc::new(AtomicBool::new(false));
        let stop_clone = Arc::clone(&stop);
        let state_clone = Arc::clone(&state);

        let (cmd_tx, mut cmd_rx) = tokio::sync::mpsc::unbounded_channel();

        let join = self.tokio_rt.spawn(async move {
            let mut pipeline = SignalPipeline::new();
            let mut t: f64 = 0.0;
            let dt = 1.0 / 256.0;
            let samples_per_packet = 12; // Muse sends 12 samples per BLE packet

            loop {
                if stop_clone.load(Ordering::SeqCst) {
                    break;
                }

                // Drain any pending commands
                while let Ok(cmd) = cmd_rx.try_recv() {
                    if let Err(reason) = pipeline.execute_command(cmd) {
                        log::warn!("Sim pipeline command rejected: {reason}");
                    }
                }

                // Generate synthetic EEG: 10 Hz alpha with per-channel
                // amplitude modulation so electrode dots visibly differ.
                // Each channel has a slow envelope (0.05-0.15 Hz) that makes
                // alpha power wax and wane independently.
                let mut frame: Vec<Vec<f32>> = Vec::with_capacity(4);
                let ch_envelope_freq = [0.07, 0.11, 0.13, 0.05]; // Hz
                let ch_base_amp = [15.0, 25.0, 20.0, 30.0]; // µV
                for ch_idx in 0..4usize {
                    let samples: Vec<f64> = (0..samples_per_packet)
                        .map(|i| {
                            let ti = t + i as f64 * dt;
                            // Slow envelope modulates alpha amplitude per channel
                            let envelope = 0.3 + 0.7
                                * (2.0 * std::f64::consts::PI * ch_envelope_freq[ch_idx] * ti)
                                    .sin()
                                    .abs();
                            let alpha = ch_base_amp[ch_idx] * envelope
                                * (2.0 * std::f64::consts::PI * 10.0 * ti).sin();
                            let noise = 3.0
                                * (2.0 * std::f64::consts::PI * 47.3 * ti + ch_idx as f64).sin();
                            alpha + noise
                        })
                        .collect();

                    if let Some(ch) = ChannelId::new(ch_idx) {
                        pipeline.ingest_samples(ch, &samples);
                        if ch == ChannelId::TP9 {
                            if let Some(analysis) = pipeline.try_analyze() {
                                state_clone.lock().unwrap().analysis = analysis;
                            }
                        }
                    }

                    frame.push(samples.iter().map(|&s| s as f32).collect());
                }

                // Feed the waveform display via push_raw_frame
                let timestamp_us = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_micros() as u64)
                    .unwrap_or(0);
                state_clone.lock().unwrap().push_raw_frame(frame, timestamp_us);

                t += samples_per_packet as f64 * dt;

                // ~256 Hz: 12 samples every ~47ms
                tokio::time::sleep(Duration::from_millis(47)).await;
            }
        });

        {
            let mut st = state.lock().unwrap();
            st.status_line = "Simulated".to_string();
            st.streaming = true;
        }

        self.connected_devices.push(ConnectedDevice {
            device_name: "Simulated Muse".to_string(),
            device_id: "sim-0000".to_string(),
            headband_id,
            state,
            stop_flag: stop,
            join_handle: Some(join),
            last_saved_fif: None,
            record_started_at: None,
            last_record_toggle: None,
            epoch_seq: 0,
            session_path: None,
            pipeline_cmd_tx: Some(cmd_tx),
        });

        self.active_tab = 0;
    }

    fn connect_to_server(&mut self) {
        let Some(ref addr) = self.server_addr else {
            return;
        };
        if self.server_state.lock().unwrap().connected {
            return;
        }

        // Create a fresh channel inside SharedOutboundTx. BLE tasks that
        // already hold a clone will pick up the new sender on their next send.
        let rx = self.outbound_tx.reset_channel();

        let n_headbands = self.connected_devices.len().max(1) as u32;

        // Collect session replay info for auto-replay of unsent frames
        let session_replays: Vec<_> = self
            .connected_devices
            .iter()
            .filter_map(|d| {
                let path = d.session_path.as_ref()?.clone();
                let frames_sent = d.state.lock().unwrap().session_frames_sent;
                Some(tcp_client::SessionReplayInfo {
                    path,
                    frames_already_sent: frames_sent,
                })
            })
            .collect();

        tcp_client::spawn_tcp_client(
            &self.tokio_rt,
            addr.clone(),
            n_headbands,
            SAMPLE_RATE_HZ,
            Arc::clone(&self.server_state),
            Arc::clone(&self.device_map),
            rx,
            session_replays,
        );
    }

    fn toggle_recording(&mut self) {
        let idx = self.active_tab;
        if idx >= self.connected_devices.len() {
            return;
        }

        let device = &mut self.connected_devices[idx];
        if let Some(t) = device.last_record_toggle {
            if t.elapsed() < Duration::from_millis(500) {
                return;
            }
        }
        device.last_record_toggle = Some(Instant::now());

        let recording = device.state.lock().unwrap().recording_active;
        if recording {
            self.stop_and_save_recording(idx);
        } else {
            device.state.lock().unwrap().start_recording();
            device.record_started_at = Some(Instant::now());
        }
    }

    fn stop_and_save_recording(&mut self, idx: usize) {
        if self.job_busy || idx >= self.connected_devices.len() {
            return;
        }

        let device = &mut self.connected_devices[idx];
        let state = Arc::clone(&device.state);
        let device_name = device.device_name.clone();
        let zuna_dir = self.zuna_dir.clone();
        let fif_path = device_fif_path(&zuna_dir, &device_name);

        device.last_saved_fif = Some(fif_path.clone());
        device.record_started_at = None;

        let (tx, rx) = mpsc::channel();
        self.job_rx = Some(rx);
        self.job_busy = true;
        self.job_device_idx = Some(idx);
        self.job_started_at = Some(Instant::now());
        self.job_last_duration = None;
        self.pipeline_step = None;
        self.pipeline_label.clear();
        self.job_log = format!("Saving recording -> {} ...\n", fif_path.display());

        let mne_script = zuna_dir.join("muse_eeg_to_fif.py");

        thread::spawn(move || {
            let (channels, n) = state.lock().unwrap().take_recording();

            if channels.len() < 4 || n < SAMPLE_RATE_HZ as usize {
                let _ = tx.send(JobEvent::LogLine(format!(
                    "Recording too short ({n} samples). Need >= {SAMPLE_RATE_HZ} (~1s at {SAMPLE_RATE_HZ} Hz).\n"
                )));
                let _ = tx.send(JobEvent::Done { ok: false });
                return;
            }

            let csv_path = std::env::temp_dir().join(format!(
                "eeg_{}.csv",
                device_name.replace(|c: char| !c.is_alphanumeric(), "_")
            ));
            let csv_result = (|| -> Result<(), String> {
                use std::io::Write;
                let f = std::fs::File::create(&csv_path)
                    .map_err(|e| format!("create {}: {e}", csv_path.display()))?;
                let mut w = std::io::BufWriter::new(f);
                writeln!(w, "TP9,AF7,AF8,TP10").map_err(|e| e.to_string())?;
                for i in 0..n {
                    writeln!(
                        w,
                        "{},{},{},{}",
                        channels[0][i], channels[1][i], channels[2][i], channels[3][i]
                    )
                    .map_err(|e| e.to_string())?;
                }
                w.flush().map_err(|e| e.to_string())?;
                Ok(())
            })();

            if let Err(e) = csv_result {
                let _ = tx.send(JobEvent::LogLine(format!("CSV write failed: {e}\n")));
                let _ = tx.send(JobEvent::Done { ok: false });
                return;
            }

            if let Err(e) = fs::create_dir_all(fif_path.parent().unwrap()) {
                let _ = tx.send(JobEvent::LogLine(format!(
                    "Failed to create output directory {}: {e}\n",
                    fif_path.parent().unwrap().display()
                )));
                let _ = tx.send(JobEvent::Done { ok: false });
                return;
            }
            let venv_py = zuna_dir.join(".venv/bin/python");
            let python = if venv_py.is_file() {
                venv_py.to_string_lossy().into_owned()
            } else {
                "python3".into()
            };

            let result = Command::new(&python)
                .arg(&mne_script)
                .arg("--csv")
                .arg(&csv_path)
                .arg("-o")
                .arg(&fif_path)
                .arg("--sfreq")
                .arg(SAMPLE_RATE_HZ.to_string())
                .output();
            let _ = std::fs::remove_file(&csv_path);

            match result {
                Ok(o) if o.status.success() => {
                    let dur_s = n as f64 / SAMPLE_RATE_HZ as f64;
                    let _ = tx.send(JobEvent::LogLine(format!(
                        "Saved {n} samples x 4 ch ({dur_s:.1} s) -> {}\n",
                        fif_path.display()
                    )));
                    let _ = tx.send(JobEvent::Done { ok: true });
                }
                Ok(o) => {
                    let stderr = String::from_utf8_lossy(&o.stderr);
                    let _ = tx.send(JobEvent::LogLine(format!(
                        "FIF conversion failed:\n{stderr}\n"
                    )));
                    let _ = tx.send(JobEvent::Done { ok: false });
                }
                Err(e) => {
                    let _ = tx.send(JobEvent::LogLine(format!("spawn {python}: {e}\n")));
                    let _ = tx.send(JobEvent::Done { ok: false });
                }
            }
        });
    }

    fn start_preprocess_for_device(&mut self) {
        if self.job_busy {
            return;
        }
        let idx = self.active_tab;
        if idx >= self.connected_devices.len() {
            return;
        }

        let fif_path = match self.connected_devices[idx].last_saved_fif.clone() {
            Some(p) if p.is_file() => p,
            _ => {
                self.muse_status =
                    "No recording to preprocess. Record first.".to_string();
                return;
            }
        };

        let zuna_dir = self.zuna_dir.clone();
        let preset = self.zuna_preset.as_cli();
        let (tx, rx) = mpsc::channel();
        self.job_rx = Some(rx);
        self.job_busy = true;
        self.job_device_idx = Some(idx);
        self.job_started_at = Some(Instant::now());
        self.job_last_duration = None;
        self.pipeline_step = None;
        self.pipeline_label.clear();
        self.job_log = format!(
            "ZUNA pipeline on {} (--preset {preset})...\n",
            fif_path.display()
        );

        thread::spawn(move || {
            let input_str = fif_path.to_string_lossy().into_owned();
            match run_zuna_pipeline_streaming(&zuna_dir, &input_str, preset, &tx) {
                Ok(pipeline_ok) => {
                    let _ = tx.send(JobEvent::Done { ok: pipeline_ok });
                }
                Err(e) => {
                    let _ = tx.send(JobEvent::LogLine(format!("ZUNA pipeline error: {e}\n")));
                    let _ = tx.send(JobEvent::Done { ok: false });
                }
            }
        });
    }

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
                Ok(JobEvent::Done { ok, .. }) => {
                    if let Some(t0) = self.job_started_at.take() {
                        self.job_last_duration = Some(t0.elapsed());
                    }
                    self.job_busy = false;
                    self.job_rx = None;
                    self.muse_status = if ok {
                        "Job finished OK".to_string()
                    } else {
                        "Job failed - see log".to_string()
                    };
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
}

// ── UI Rendering ────────────────────────────────────────────────────────────

impl eframe::App for EegViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint();
        self.poll_job();
        self.poll_scan();

        let is_multi_muse = self.input_kind == InputKind::MuseBle;

        if is_multi_muse {
            self.render_multi_device_ui(ctx);
        } else {
            self.render_legacy_ui(ctx);
        }
    }
}

impl EegViewerApp {
    fn render_multi_device_ui(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            // ── Tab bar ──
            ui.horizontal(|ui| {
                let mut tabs_to_render: Vec<(usize, String, bool, bool)> = Vec::new();
                for (idx, device) in self.connected_devices.iter().enumerate() {
                    let st = device.state.lock().unwrap();
                    tabs_to_render.push((
                        idx,
                        device.device_name.clone(),
                        st.streaming,
                        st.recording_active,
                    ));
                }

                for (idx, name, streaming, recording) in &tabs_to_render {
                    let dot = match (streaming, recording) {
                        (_, true) => "R",
                        (true, _) => "S",
                        _ => "x",
                    };
                    let selected = *idx == self.active_tab;
                    let label = format!("[{dot}] {name}");
                    if ui
                        .selectable_label(selected, &label)
                        .clicked()
                    {
                        self.active_tab = *idx;
                    }
                }

                ui.separator();

                if self.scan_active {
                    ui.spinner();
                    ui.label("Scanning...");
                } else if ui.button("Scan").clicked() {
                    self.start_scan();
                }

                if self.connected_devices.len() < 4 && !self.scan_results.is_empty() {
                    ui.separator();
                    ui.label("Devices:");
                }
            });

            // ── Device picker (inline after scan) ──
            if !self.scan_results.is_empty() && self.show_device_picker {
                let connected_ids: HashSet<&str> = self
                    .connected_devices
                    .iter()
                    .map(|d| d.device_id.as_str())
                    .collect();

                let available: Vec<(usize, String)> = self
                    .scan_results
                    .iter()
                    .enumerate()
                    .filter(|(_, sr)| !connected_ids.contains(sr.device.id.as_str()))
                    .map(|(i, sr)| (i, sr.display_label.clone()))
                    .collect();

                if !available.is_empty() {
                    ui.horizontal(|ui| {
                        ui.label("Found:");
                        let mut connect_idx = None;
                        for (idx, label) in &available {
                            if ui.button(label).clicked() {
                                connect_idx = Some(*idx);
                            }
                        }
                        if let Some(idx) = connect_idx {
                            self.connect_device(idx);
                        }
                        if ui.button("Dismiss").clicked() {
                            self.show_device_picker = false;
                        }
                    });
                }
            }

            // ── Server status ──
            ui.horizontal(|ui| {
                let (connected, status) = {
                    let st = self.server_state.lock().unwrap();
                    (st.connected, st.status_line.clone())
                };

                if connected {
                    ui.colored_label(
                        egui::Color32::from_rgb(80, 200, 120),
                        format!("Server: {status}"),
                    );
                } else {
                    ui.label(RichText::new(format!("Server: {status}")).weak());
                    if self.server_addr.is_some() && ui.button("Connect Server").clicked() {
                        self.connect_to_server();
                    }
                }

                ui.separator();

                // ── Per-device toolbar ──
                if self.active_tab < self.connected_devices.len() {
                    let (recording, sample_count, streaming) = {
                        let st = self.connected_devices[self.active_tab]
                            .state
                            .lock()
                            .unwrap();
                        (st.recording_active, st.record_sample_count, st.streaming)
                    };

                    if recording {
                        let elapsed = self.connected_devices[self.active_tab]
                            .record_started_at
                            .map(|t| format_duration(t.elapsed()))
                            .unwrap_or_default();
                        let btn = egui::Button::new(
                            RichText::new(format!(
                                "Stop ({elapsed}, {sample_count} samples)"
                            ))
                            .color(egui::Color32::RED),
                        );
                        if ui.add(btn).clicked() {
                            self.toggle_recording();
                        }
                    } else {
                        let can_record = streaming && !self.job_busy;
                        if ui
                            .add_enabled(can_record, egui::Button::new("Record"))
                            .clicked()
                        {
                            self.toggle_recording();
                        }
                    }

                    let has_fif = self.connected_devices[self.active_tab]
                        .last_saved_fif
                        .as_ref()
                        .map_or(false, |p| p.is_file());
                    if ui
                        .add_enabled(
                            has_fif && !self.job_busy && !recording,
                            egui::Button::new("Preprocess"),
                        )
                        .clicked()
                    {
                        self.start_preprocess_for_device();
                    }

                    // Disconnect button
                    if ui.button("Disconnect").clicked() {
                        self.disconnect_device(self.active_tab);
                    }
                }
            });

            // ── Display toggles ──
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.show_raw, "Raw (blue)");
                ui.checkbox(&mut self.show_reconstructed, "Reconstructed (red)");

                ui.separator();
                ui.selectable_value(&mut self.viz_mode, VizMode::Waveforms, "Waveforms");
                ui.selectable_value(&mut self.viz_mode, VizMode::NeuralAura, "Neural Aura");
                ui.selectable_value(&mut self.viz_mode, VizMode::BrainTopography, "Brain Topo");

                if self.job_busy {
                    ui.spinner();
                    if let Some(t0) = self.job_started_at {
                        ui.label(format!("elapsed: {}", format_duration(t0.elapsed())));
                    }
                } else if let Some(d) = self.job_last_duration {
                    ui.label(format!("last: {}", format_duration(d)));
                }
            });

            // ── Dimension controls, visible when not in Waveforms mode ──
            if self.viz_mode != VizMode::Waveforms
                && self.active_tab < self.connected_devices.len()
            {
                let device = &self.connected_devices[self.active_tab];
                let analysis = device.state.lock().unwrap().analysis.clone();
                if let Some(ref cmd_tx) = device.pipeline_cmd_tx {
                    ui.horizontal(|ui| {
                        render_dimension_controls(ui, &analysis, cmd_tx);
                    });
                }
            }

            // ── Job log ──
            if !self.job_log.is_empty() {
                egui::ScrollArea::vertical()
                    .max_height(120.0)
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new(&self.job_log).monospace().size(11.0),
                        );
                    });
            }
        });

        // ── Bottom status bar ──
        egui::TopBottomPanel::bottom("status_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                // Server status
                let (srv_connected, srv_status) = {
                    let st = self.server_state.lock().unwrap();
                    (st.connected, st.status_line.clone())
                };
                if srv_connected {
                    ui.colored_label(
                        egui::Color32::from_rgb(80, 200, 120),
                        format!("Server: {srv_status}"),
                    );
                } else {
                    ui.label(RichText::new(format!("Server: {srv_status}")).weak());
                }

                ui.separator();

                // Per-device packet counters
                if self.active_tab < self.connected_devices.len() {
                    let st = self.connected_devices[self.active_tab]
                        .state
                        .lock()
                        .unwrap();
                    ui.label(format!(
                        "Local: {} frames",
                        st.raw_frame_count,
                    ));
                    ui.separator();
                    ui.label(format!(
                        "Disk: {} written",
                        st.session_frames_written,
                    ));
                    ui.separator();
                    ui.label(format!(
                        "Server: {} sent",
                        st.session_frames_sent,
                    ));
                    let unsent = st
                        .session_frames_written
                        .saturating_sub(st.session_frames_sent);
                    if unsent > 0 {
                        ui.separator();
                        ui.colored_label(
                            egui::Color32::from_rgb(255, 180, 0),
                            format!("{unsent} pending"),
                        );
                    }
                    if st.recon_frame_count > 0 {
                        ui.separator();
                        ui.label(format!(
                            "Recon: {} frames",
                            st.recon_frame_count,
                        ));
                    }
                } else {
                    ui.label("No device selected");
                }
            });
        });

        // ── Neural Aura: right side panel ──
        if self.viz_mode == VizMode::NeuralAura
            && self.active_tab < self.connected_devices.len()
        {
            let analysis = self.connected_devices[self.active_tab]
                .state
                .lock()
                .unwrap()
                .analysis
                .clone();
            egui::SidePanel::right("neural_aura_panel")
                .exact_width(280.0)
                .show(ctx, |ui| {
                    viz_aura::render_neural_aura(ui, &analysis.dimensions);
                });
        }

        // ── Brain Topography: bottom panel ──
        if self.viz_mode == VizMode::BrainTopography
            && self.active_tab < self.connected_devices.len()
        {
            let analysis = self.connected_devices[self.active_tab]
                .state
                .lock()
                .unwrap()
                .analysis
                .clone();
            egui::TopBottomPanel::bottom("brain_topo_panel")
                .resizable(true)
                .default_height(ctx.screen_rect().height() * 0.4)
                .show(ctx, |ui| {
                    viz_topo::render_brain_topography(ui, &analysis);
                });
        }

        // ── Central panel: EEG plot for active device ──
        egui::CentralPanel::default().show(ctx, |ui| {
            if self.connected_devices.is_empty() {
                ui.vertical_centered(|ui| {
                    ui.add_space(40.0);
                    ui.label("No devices connected. Click Scan to find Muse headsets.");
                });
                return;
            }

            if self.active_tab >= self.connected_devices.len() {
                return;
            }

            let (raw_snapshot, recon_snapshot) = {
                let st = self.connected_devices[self.active_tab]
                    .state
                    .lock()
                    .unwrap();
                (st.raw_channels.clone(), st.recon_channels.clone())
            };

            let n_ch = raw_snapshot.len();
            if n_ch == 0 {
                ui.vertical_centered(|ui| {
                    let st = self.connected_devices[self.active_tab]
                        .state
                        .lock()
                        .unwrap();
                    ui.label(&st.status_line);
                });
                return;
            }

            let has_recon = !recon_snapshot.is_empty() && recon_snapshot.len() == n_ch;
            let plot_h = (ui.available_height() / n_ch as f32).clamp(72.0, 220.0);

            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    for i in 0..n_ch {
                        let label = channel_label(None, n_ch, i);
                        ui.horizontal(|ui| {
                            ui.allocate_ui_with_layout(
                                egui::vec2(52.0, plot_h),
                                egui::Layout::top_down(egui::Align::Center),
                                |ui| {
                                    ui.add_space(plot_h * 0.35);
                                    ui.label(
                                        RichText::new(&label)
                                            .strong()
                                            .monospace()
                                            .size(13.0),
                                    );
                                },
                            );

                            Plot::new(format!("eeg_ch_{i}"))
                                .height(plot_h)
                                .width(ui.available_width())
                                .allow_zoom(egui::Vec2b::new(true, false))
                                .allow_drag(egui::Vec2b::new(true, false))
                                .allow_scroll(egui::Vec2b::new(true, false))
                                .show_axes(true)
                                .show_grid(true)
                                .legend(
                                    Legend::default()
                                        .position(egui_plot::Corner::RightTop),
                                )
                                .show(ui, |plot_ui| {
                                    if self.show_raw {
                                        let ch = &raw_snapshot[i];
                                        let pts: PlotPoints = ch.iter().copied().collect();
                                        plot_ui.line(
                                            Line::new(pts)
                                                .name("Raw")
                                                .color(COLOR_ORIGINAL),
                                        );
                                    }

                                    if has_recon && self.show_reconstructed {
                                        let ch = &recon_snapshot[i];
                                        let pts: PlotPoints = ch.iter().copied().collect();
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
                });
        });
    }

    // ── Legacy UI (stdin/TCP mode) ──────────────────────────────────────

    fn render_legacy_ui(&mut self, ctx: &egui::Context) {
        let legacy = match self.legacy_state.as_ref() {
            Some(s) => Arc::clone(s),
            None => return,
        };

        let (n_ch, frames, last_error, tcp_connected, recording_active, record_sample_count) = {
            let st = legacy.lock().unwrap();
            (
                st.channels_orig.len(),
                st.frames_received,
                st.last_error.clone(),
                st.tcp_client_connected,
                st.recording_active,
                st.record_sample_count,
            )
        };

        let is_tcp_or_stdin = matches!(self.input_kind, InputKind::Tcp | InputKind::Stdin);

        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.show_live, "Show live");
                ui.separator();

                if is_tcp_or_stdin {
                    if recording_active {
                        let btn = egui::Button::new(
                            RichText::new("Stop Recording")
                                .color(egui::Color32::RED),
                        );
                        if ui.add_enabled(!self.job_busy, btn).clicked() {
                            let mut st = legacy.lock().unwrap();
                            st.recording_active = false;
                        }
                    } else {
                        if ui
                            .add_enabled(!self.job_busy, egui::Button::new("Record"))
                            .clicked()
                        {
                            let mut st = legacy.lock().unwrap();
                            st.record_buffer.clear();
                            st.record_sample_count = 0;
                            st.recording_active = true;
                        }
                    }
                }

                if self.job_busy {
                    ui.spinner();
                }
            });
            ui.horizontal(|ui| {
                ui.label("Source:");
                match self.input_kind {
                    InputKind::Stdin => {
                        ui.label("stdin");
                    }
                    InputKind::Tcp => {
                        if tcp_connected {
                            ui.colored_label(
                                egui::Color32::from_rgb(80, 200, 120),
                                "TCP client",
                            );
                        } else {
                            ui.label(RichText::new("TCP listening...").weak());
                        }
                    }
                    InputKind::None => {
                        ui.label(RichText::new("off").weak());
                    }
                    InputKind::MuseBle => {} // handled by multi-device UI
                }
                ui.separator();
                if recording_active {
                    ui.colored_label(
                        egui::Color32::from_rgb(220, 50, 50),
                        format!(
                            "REC {} samples ({:.1} s)",
                            record_sample_count,
                            record_sample_count as f64 / SAMPLE_RATE_HZ as f64
                        ),
                    );
                }
                ui.label(RichText::new(&self.muse_status).weak());
            });
            if n_ch > 0 {
                ui.horizontal(|ui| {
                    ui.label(format!("channels: {n_ch}  frames: {frames}"));
                });
            }
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.show_raw, "Original (blue)");
                ui.checkbox(&mut self.show_reconstructed, "Reconstructed (red)");
            });
            if let Some(ref e) = last_error {
                ui.colored_label(egui::Color32::RED, e);
            }
            if !self.job_log.is_empty() {
                egui::ScrollArea::vertical()
                    .max_height(160.0)
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new(&self.job_log).monospace().size(11.0),
                        );
                    });
            }
        });

        // Snapshot data and drop lock before rendering
        let (orig_snapshot, recon_snapshot, channel_names_snapshot) = {
            let st = legacy.lock().unwrap();
            (
                st.channels_orig.clone(),
                st.channels_recon.clone(),
                st.channel_names.clone(),
            )
        };

        egui::CentralPanel::default().show(ctx, |ui| {
            if !self.show_live || n_ch == 0 {
                ui.vertical_centered(|ui| {
                    ui.label("Waiting for data...");
                });
                return;
            }

            let has_recon =
                !recon_snapshot.is_empty() && recon_snapshot.len() == orig_snapshot.len();
            let plot_h = (ui.available_height() / n_ch as f32).clamp(72.0, 220.0);

            egui::ScrollArea::vertical()
                .auto_shrink([false, false])
                .show(ui, |ui| {
                    let names = channel_names_snapshot.as_deref();
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
                                .allow_zoom(egui::Vec2b::new(true, false))
                                .allow_drag(egui::Vec2b::new(true, false))
                                .allow_scroll(egui::Vec2b::new(true, false))
                                .show_axes(true)
                                .show_grid(true)
                                .legend(
                                    Legend::default()
                                        .position(egui_plot::Corner::RightTop),
                                )
                                .show(ui, |plot_ui| {
                                    if self.show_raw && i < orig_snapshot.len() {
                                        let ch = &orig_snapshot[i];
                                        let pts: PlotPoints = ch
                                            .iter()
                                            .enumerate()
                                            .map(|(t, &v)| [t as f64, v as f64])
                                            .collect();
                                        plot_ui.line(
                                            Line::new(pts)
                                                .name("Original")
                                                .color(COLOR_ORIGINAL),
                                        );
                                    }
                                    if has_recon && self.show_reconstructed && i < recon_snapshot.len() {
                                        let ch = &recon_snapshot[i];
                                        let pts: PlotPoints = ch
                                            .iter()
                                            .enumerate()
                                            .map(|(t, &v)| [t as f64, v as f64])
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
                });
        });
    }
}

// ── Helper functions ────────────────────────────────────────────────────────

fn render_dimension_controls(
    ui: &mut egui::Ui,
    analysis: &AnalysisFrame,
    cmd_tx: &tokio::sync::mpsc::UnboundedSender<PipelineCommand>,
) {
    // Baseline button
    match analysis.baseline_state() {
        BaselineState::NotCollected | BaselineState::Failed(_) => {
            if ui.button("Start Baseline").clicked() {
                let _ = cmd_tx.send(PipelineCommand::StartBaseline);
            }
        }
        BaselineState::Collecting { progress } => {
            if ui
                .button(format!("Stop Baseline ({:.0}%)", progress * 100.0))
                .clicked()
            {
                let _ = cmd_tx.send(PipelineCommand::StopBaseline);
            }
        }
        BaselineState::Collected => {
            ui.add_enabled(false, egui::Button::new("Baseline done"));
        }
    }

    ui.separator();

    // Dimension start/stop buttons
    let dims: [(
        &str,
        &DimensionReading,
        PipelineCommand,
        PipelineCommand,
        bool,
    ); 4] = [
        (
            "Absorption",
            &analysis.dimensions[0].reading,
            PipelineCommand::StartAbsorption,
            PipelineCommand::StopAbsorption,
            true, // needs baseline
        ),
        (
            "Attunement",
            &analysis.dimensions[1].reading,
            PipelineCommand::StartEntrainment,
            PipelineCommand::StopEntrainment,
            false,
        ),
        (
            "Unknown",
            &analysis.dimensions[2].reading,
            PipelineCommand::StartUnknown,
            PipelineCommand::StopUnknown,
            false,
        ),
        (
            "Witnessed",
            &analysis.dimensions[3].reading,
            PipelineCommand::StartWitnessed,
            PipelineCommand::StopWitnessed,
            false,
        ),
    ];

    for (name, reading, start_cmd, stop_cmd, needs_baseline) in &dims {
        let enabled = !needs_baseline || analysis.baseline_available;
        match reading {
            DimensionReading::Idle => {
                let btn = egui::Button::new(format!("Start {name}"));
                if ui.add_enabled(enabled, btn).clicked() {
                    let _ = cmd_tx.send(*start_cmd);
                }
            }
            DimensionReading::Settling { progress }
            | DimensionReading::Measuring { progress, .. } => {
                if ui
                    .button(format!("Stop {name} ({:.0}%)", progress * 100.0))
                    .clicked()
                {
                    let _ = cmd_tx.send(*stop_cmd);
                }
            }
            DimensionReading::Complete(_) => {
                ui.add_enabled(false, egui::Button::new(format!("{name} done")));
            }
        }
    }
}

fn format_duration(d: Duration) -> String {
    let total_secs = d.as_secs();
    if total_secs >= 3600 {
        let h = total_secs / 3600;
        let m = (total_secs % 3600) / 60;
        let s = total_secs % 60;
        format!("{h}h {m}m {s}s")
    } else if total_secs >= 60 {
        let m = total_secs / 60;
        let s = total_secs % 60;
        format!("{m}m {s}s")
    } else {
        let secs = d.as_secs_f64();
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

// ── Legacy frame reading (stdin/TCP EEGF/EEGD) ─────────────────────────────

fn read_u32<R: Read>(r: &mut R) -> io::Result<u32> {
    let mut b = [0u8; 4];
    r.read_exact(&mut b)?;
    Ok(u32::from_le_bytes(b))
}

enum FrameKind {
    Single(Vec<Vec<f32>>),
    Dual { orig: Vec<Vec<f32>>, recon: Vec<Vec<f32>> },
}

fn floats_to_channel_major(out: Vec<f32>, n_channels: u32, n_samples: u32) -> io::Result<Vec<Vec<f32>>> {
    let total = (n_channels as usize) * (n_samples as usize);
    if out.len() != total {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "float buffer length mismatch"));
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
        return Err(io::Error::new(io::ErrorKind::InvalidData, "n_channels and n_samples must be > 0"));
    }
    let total = (n_channels as usize)
        .checked_mul(n_samples as usize)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "size overflow"))?;
    let nbytes = total.checked_mul(4)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "size overflow"))?;

    match magic {
        MAGIC_SINGLE => {
            buf.resize(nbytes, 0);
            r.read_exact(buf)?;
            let mut out = Vec::with_capacity(total);
            for chunk in buf.chunks_exact(4) { out.push(f32::from_le_bytes(chunk.try_into().unwrap())); }
            let by_channel = floats_to_channel_major(out, n_channels, n_samples)?;
            Ok(Some(FrameKind::Single(by_channel)))
        }
        MAGIC_DUAL => {
            let nbytes_dual = nbytes.checked_mul(2)
                .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "size overflow"))?;
            buf.resize(nbytes_dual, 0);
            r.read_exact(buf)?;
            let mut out = Vec::with_capacity(total * 2);
            for chunk in buf.chunks_exact(4) { out.push(f32::from_le_bytes(chunk.try_into().unwrap())); }
            let orig = floats_to_channel_major(out[..total].to_vec(), n_channels, n_samples)?;
            let recon = floats_to_channel_major(out[total..].to_vec(), n_channels, n_samples)?;
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
        v.drain(0..v.len() - max_points);
    }
}

fn merge_stack(acc: &mut Vec<Vec<f32>>, incoming: Vec<Vec<f32>>, max_points: usize) {
    if acc.is_empty() || acc.len() != incoming.len() {
        let mut by_channel = incoming;
        for v in &mut by_channel { trim_front(v, max_points); }
        *acc = by_channel;
    } else {
        for (a, chunk) in acc.iter_mut().zip(incoming.iter()) {
            a.extend_from_slice(chunk);
            trim_front(a, max_points);
        }
    }
}

fn merge_single(state: &mut SharedState, by_channel: Vec<Vec<f32>>, max_points: usize) {
    state.channels_recon.clear();
    state.channel_names = None;
    if state.recording_active {
        if state.record_buffer.is_empty() || state.record_buffer.len() != by_channel.len() {
            state.record_buffer = by_channel.clone();
        } else {
            for (acc, incoming) in state.record_buffer.iter_mut().zip(by_channel.iter()) {
                acc.extend_from_slice(incoming);
            }
        }
        state.record_sample_count = state.record_buffer.iter().map(|v| v.len()).min().unwrap_or(0);
    }
    merge_stack(&mut state.channels_orig, by_channel, max_points);
    state.frames_received = state.frames_received.saturating_add(1);
}

fn merge_dual(state: &mut SharedState, orig: Vec<Vec<f32>>, recon: Vec<Vec<f32>>, max_points: usize) {
    if orig.len() != recon.len() {
        state.last_error = Some(format!("dual frame: orig ch {} != recon ch {}", orig.len(), recon.len()));
        return;
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
                state.lock().unwrap().last_error = Some(format!("read error: {e}"));
                break;
            }
        }
    }
    Ok(())
}

// ── Main ────────────────────────────────────────────────────────────────────

fn main() -> eframe::Result<()> {
    env_logger::Builder::from_env(
        env_logger::Env::default().default_filter_or("eeg_viewer=info"),
    )
    .init();

    let args = Args::parse();
    let input_kind = resolve_input_kind(&args);
    let max_points = args.max_points;

    let tokio_rt = Arc::new(
        tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("tokio runtime"),
    );

    let legacy_state: Option<Arc<Mutex<SharedState>>> = match input_kind {
        InputKind::Tcp | InputKind::Stdin | InputKind::None => {
            Some(Arc::new(Mutex::new(SharedState::default())))
        }
        InputKind::MuseBle => None,
    };

    // Legacy stdin/TCP setup
    if let Some(ref ls) = legacy_state {
        match input_kind {
            InputKind::Tcp => {
                let addr = args.tcp.as_ref().expect("tcp");
                let sa: SocketAddr = addr.parse().unwrap_or_else(|e| {
                    eprintln!("bad --tcp address: {e}");
                    std::process::exit(1);
                });
                let state_tcp = Arc::clone(ls);
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
                        let _ = read_frames_from(&mut stream, &mut raw, &state_tcp, max_points);
                    }
                });
            }
            InputKind::Stdin => {
                let state_in = Arc::clone(ls);
                thread::spawn(move || {
                    let stdin = io::stdin();
                    let mut locked = stdin.lock();
                    let mut raw = Vec::new();
                    let _ = read_frames_from(&mut locked, &mut raw, &state_in, max_points);
                });
            }
            _ => {}
        }
    }

    let muse_status = match input_kind {
        InputKind::MuseBle => "Ready - click Scan".to_string(),
        InputKind::Stdin => "stdin mode".to_string(),
        InputKind::Tcp => "TCP mode".to_string(),
        InputKind::None => "idle".to_string(),
    };

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("EEG Viewer - Multi-Muse")
            .with_inner_size([1024.0, 720.0]),
        ..Default::default()
    };

    let server_state = Arc::new(Mutex::new(ServerState::default()));
    let zuna_dir = args.zuna_dir.clone();
    let muse_record_bin = args.muse_record_bin.clone();
    let server_addr = args.server.clone();
    let simulate = args.simulate;

    eframe::run_native(
        "eeg-viewer",
        options,
        Box::new(move |_cc| {
            let mut app = EegViewerApp {
                connected_devices: Vec::new(),
                active_tab: 0,
                scan_results: Vec::new(),
                scan_active: false,
                scan_rx: None,
                show_device_picker: false,
                server_state,
                server_addr,
                outbound_tx: SharedOutboundTx::new(),
                device_map: Arc::new(Mutex::new(std::collections::HashMap::new())),
                show_live: true,
                show_raw: true,
                show_reconstructed: true,
                viz_mode: VizMode::default(),
                zuna_dir,
                muse_record_bin,
                zuna_preset: ZunaPreset::default(),
                record_secs: 120,
                job_busy: false,
                job_log: String::new(),
                job_rx: None,
                job_device_idx: None,
                job_started_at: None,
                job_last_duration: None,
                pipeline_step: None,
                pipeline_label: String::new(),
                max_points,
                input_kind,
                tokio_rt,
                legacy_state,
                muse_status,
                resume_muse_pending: false,
                reset_confirm_pending: false,
            };
            if simulate {
                app.spawn_simulated_device();
            }
            Ok(Box::new(app) as Box<dyn eframe::App>)
        }),
    )
}
