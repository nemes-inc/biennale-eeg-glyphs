//! Multi-device Muse BLE connection manager.
//!
//! Each connected Muse gets its own tokio task via [`spawn_device_ble_task`].
//! Uses `scan_all()` + `connect_to()` to avoid creating multiple CBCentralManagers.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::Result;
use log::{error, info, warn};
use muse_rs::muse_client::{MuseClient, MuseClientConfig, MuseDevice};
use muse_rs::types::MuseEvent;

use crate::device_state::{
    build_eegm_frame, chunk_ready, drain_chunk, DeviceState, HeadbandId, SessionWriter,
    SharedOutboundTx,
};
use crate::signal_pipeline::{ChannelId, PipelineCommand, SignalPipeline};

/// Samples to accumulate before flushing to shared state.
const CHUNK: usize = 256;

/// Number of EEG channels per Muse headband.
const CHANNELS: usize = 4;

/// Handle to a running BLE connection task.
pub struct DeviceBleHandle {
    pub stop: Arc<AtomicBool>,
    pub join: tokio::task::JoinHandle<()>,
}

impl DeviceBleHandle {
    #[allow(dead_code)]
    pub fn request_stop(&self) {
        self.stop.store(true, Ordering::SeqCst);
    }
}

/// Scan result wrapping a MuseDevice with a display-friendly label.
pub struct ScanResult {
    pub device: MuseDevice,
    pub display_label: String,
}

/// Spawn a background scan for Muse devices.
/// Returns a channel receiver that delivers scan results once.
/// Err variant surfaces scan failures to the UI (vs. Ok(empty) = no devices found).
pub fn spawn_scan_task(
    rt: &tokio::runtime::Runtime,
    scan_timeout_secs: u64,
) -> std::sync::mpsc::Receiver<Result<Vec<ScanResult>, String>> {
    let (tx, rx) = std::sync::mpsc::channel();
    rt.spawn(async move {
        let config = MuseClientConfig {
            scan_timeout_secs,
            name_prefix: "Muse".to_string(),
            ..Default::default()
        };
        let client = MuseClient::new(config);
        let result = match client.scan_all().await {
            Ok(devices) => Ok(devices
                .into_iter()
                .map(|d| {
                    let label =
                        crate::device_state::device_display_label(&d.name, &d.id);
                    ScanResult {
                        device: d,
                        display_label: label,
                    }
                })
                .collect()),
            Err(e) => {
                error!("Scan failed: {e}");
                Err(format!("Scan failed: {e}"))
            }
        };
        let _ = tx.send(result);
    });
    rx
}

/// Spawn a BLE streaming task for one specific Muse device.
/// Accepts a command receiver for UI to pipeline communication.
pub fn spawn_device_ble_task(
    device: MuseDevice,
    headband_id: HeadbandId,
    state: Arc<Mutex<DeviceState>>,
    outbound_tx: SharedOutboundTx,
    session_path: Option<PathBuf>,
    cmd_rx: tokio::sync::mpsc::UnboundedReceiver<PipelineCommand>,
    rt: &tokio::runtime::Runtime,
) -> DeviceBleHandle {
    let stop = Arc::new(AtomicBool::new(false));
    let stop_clone = Arc::clone(&stop);

    let join = rt.spawn(async move {
        match run_device_ble_loop(
            device,
            headband_id,
            state.clone(),
            stop_clone,
            outbound_tx,
            session_path,
            cmd_rx,
        )
        .await
        {
            Ok(()) => info!("BLE loop ended cleanly"),
            Err(e) => {
                error!("BLE error: {e:#}");
                let mut st = state.lock().unwrap();
                st.status_line = format!("Error: {e}");
                st.streaming = false;
            }
        }
    });

    DeviceBleHandle { stop, join }
}

async fn run_device_ble_loop(
    device: MuseDevice,
    headband_id: HeadbandId,
    state: Arc<Mutex<DeviceState>>,
    stop: Arc<AtomicBool>,
    outbound_tx: SharedOutboundTx,
    session_path: Option<PathBuf>,
    mut cmd_rx: tokio::sync::mpsc::UnboundedReceiver<PipelineCommand>,
) -> Result<()> {
    {
        let mut st = state.lock().unwrap();
        st.status_line = "Connecting…".to_string();
        st.streaming = false;
    }

    // NOTE: MuseClient is required as a method receiver for connect_to(),
    // but connect_to() uses device.adapter (from scan), not the client's adapter.
    // This creates an unused CBCentralManager on macOS — a muse_rs API limitation.
    let client = MuseClient::new(MuseClientConfig::default());
    let (mut rx, handle) = client.connect_to(device).await?;
    handle.start(false, false).await?;

    {
        let mut st = state.lock().unwrap();
        st.status_line = "Streaming".to_string();
        st.streaming = true;
    }

    // Open session file for write-ahead logging
    let mut session_writer = match session_path {
        Some(ref path) => match SessionWriter::create(path) {
            Ok(sw) => {
                info!("Session file: {}", path.display());
                Some(sw)
            }
            Err(e) => {
                warn!("Failed to create session file {}: {e}", path.display());
                None
            }
        },
        None => None,
    };

    let mut channel_accum: Vec<Vec<f32>> = vec![Vec::new(); CHANNELS];
    let mut epoch_seq: u32 = 0;
    let mut pipeline = SignalPipeline::new();

    loop {
        if stop.load(Ordering::SeqCst) {
            break;
        }

        tokio::select! {
            evt = rx.recv() => {
                let Some(evt) = evt else { break };

                match evt {
                    MuseEvent::Eeg(r) if (r.electrode as usize) < CHANNELS => {
                        // Feed signal pipeline with original f64 samples
                        if let Some(ch) = ChannelId::new(r.electrode as usize) {
                            pipeline.ingest_samples(ch, &r.samples);
                            // Run analysis on TP9 to avoid multi-channel double-trigger
                            if ch == ChannelId::TP9 {
                                if let Some(frame) = pipeline.try_analyze() {
                                    state.lock().unwrap().analysis = frame;
                                }
                            }
                        }

                        // Existing EEGM frame path, casts to f32 independently
                        channel_accum[r.electrode as usize]
                            .extend(r.samples.iter().map(|&s| s as f32));

                        while chunk_ready(&channel_accum, CHUNK) {
                            if stop.load(Ordering::SeqCst) { break; }

                            let frame = drain_chunk(&mut channel_accum, CHUNK);
                            let timestamp_us = std::time::SystemTime::now()
                                .duration_since(std::time::UNIX_EPOCH)
                                .map(|d| d.as_micros() as u64)
                                .unwrap_or(0);
                            let eegm = build_eegm_frame(headband_id, epoch_seq, &frame, timestamp_us);

                            // Write-ahead: persist to session file FIRST
                            if let Some(ref mut sw) = session_writer {
                                if let Err(e) = sw.append_frame(&eegm) {
                                    warn!("Session write failed: {e}");
                                }
                            }

                            // Then send to inference server
                            let sent = outbound_tx.send(eegm);
                            if epoch_seq < 3 || epoch_seq % 100 == 0 {
                                info!("BLE outbound send seq={epoch_seq} sent={sent}");
                            }
                            epoch_seq = epoch_seq.wrapping_add(1);

                            // Update shared state for UI and counters
                            {
                                let mut st = state.lock().unwrap();
                                st.push_raw_frame(frame, timestamp_us);
                                if let Some(ref sw) = session_writer {
                                    st.session_frames_written = sw.frames_written();
                                }
                            }
                        }
                    }
                    MuseEvent::Connected(name) => {
                        info!("Muse BLE connected: {name}");
                        let mut st = state.lock().unwrap();
                        st.status_line = format!("Connected: {name}");
                        st.streaming = true;
                    }
                    MuseEvent::Disconnected => {
                        warn!("BLE disconnected");
                        let mut st = state.lock().unwrap();
                        st.status_line = "Disconnected".to_string();
                        st.streaming = false;
                        break;
                    }
                    _ => {}
                }
            }
            // Pipeline commands from UI thread
            Some(cmd) = cmd_rx.recv() => {
                if let Err(reason) = pipeline.execute_command(cmd) {
                    warn!("Pipeline command rejected: {reason}");
                }
            }
            _ = tokio::time::sleep(Duration::from_millis(50)) => {
                if stop.load(Ordering::SeqCst) {
                    break;
                }
            }
        }
    }

    let _ = handle.disconnect().await;
    {
        let mut st = state.lock().unwrap();
        st.streaming = false;
    }
    Ok(())
}
