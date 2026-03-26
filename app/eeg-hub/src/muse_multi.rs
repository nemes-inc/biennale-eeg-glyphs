//! Multi-headband Muse BLE connection manager.
//!
//! Scans for up to [`MAX_HEADBANDS`] Muse devices, connects to each in its own
//! tokio task, and feeds EEG samples into per-headband ring buffers shared with
//! the UI and TCP sender.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::Result;
use log::{error, info, warn};
use muse_rs::muse_client::{MuseClient, MuseClientConfig};
use muse_rs::types::MuseEvent;

use crate::HeadbandState;

/// Maximum headbands supported.
pub const MAX_HEADBANDS: usize = 4;

/// Number of EEG channels per Muse headband.
pub const CHANNELS_PER_HEADBAND: usize = 4;

/// Samples to accumulate before flushing to shared state.
const CHUNK: usize = 256;

/// Handle to one running BLE connection task.
pub struct MuseHandle {
    pub stop: Arc<AtomicBool>,
    pub join: tokio::task::JoinHandle<()>,
}

impl MuseHandle {
    pub fn request_stop(&self) {
        self.stop.store(true, Ordering::SeqCst);
    }
}

/// Spawn a BLE connection task for one headband.
///
/// The task scans for a Muse with the given `name_prefix`, connects, and
/// streams EEG into `state.raw_buffers[headband_id]`.
pub fn spawn_headband_task(
    headband_id: usize,
    state: Arc<Mutex<HeadbandState>>,
    name_prefix: String,
    scan_timeout_secs: u64,
) -> MuseHandle {
    let stop = Arc::new(AtomicBool::new(false));
    let stop_clone = Arc::clone(&stop);

    let join = tokio::spawn(async move {
        match run_ble_loop(headband_id, state.clone(), stop_clone, &name_prefix, scan_timeout_secs)
            .await
        {
            Ok(()) => info!("Headband {headband_id}: BLE loop ended cleanly"),
            Err(e) => {
                error!("Headband {headband_id}: BLE error: {e:#}");
                let mut st = state.lock().unwrap();
                st.status[headband_id] = format!("Error: {e}");
                st.connected[headband_id] = false;
            }
        }
    });

    MuseHandle { stop, join }
}

async fn run_ble_loop(
    headband_id: usize,
    state: Arc<Mutex<HeadbandState>>,
    stop: Arc<AtomicBool>,
    name_prefix: &str,
    scan_timeout_secs: u64,
) -> Result<()> {
    {
        let mut st = state.lock().unwrap();
        st.status[headband_id] = "Scanning…".to_string();
        st.connected[headband_id] = false;
    }

    let config = MuseClientConfig {
        enable_aux: false,
        enable_ppg: false,
        scan_timeout_secs,
        name_prefix: name_prefix.to_string(),
    };

    let client = MuseClient::new(config);
    let (mut rx, handle) = client.connect().await?;
    handle.start(false, false).await?;

    {
        let mut st = state.lock().unwrap();
        st.status[headband_id] = "Connected".to_string();
        st.connected[headband_id] = true;
    }

    let mut buffers: Vec<Vec<f32>> = vec![Vec::new(); CHANNELS_PER_HEADBAND];

    loop {
        tokio::select! {
            evt = rx.recv() => {
                if stop.load(Ordering::SeqCst) {
                    break;
                }
                match evt {
                    Some(MuseEvent::Eeg(r)) if r.electrode < CHANNELS_PER_HEADBAND => {
                        buffers[r.electrode].extend(r.samples.iter().map(|&s| s as f32));
                        while buffers.iter().map(|v| v.len()).min().unwrap_or(0) >= CHUNK {
                            let frame: Vec<Vec<f32>> = buffers
                                .iter_mut()
                                .map(|b| b.drain(0..CHUNK).collect())
                                .collect();
                            let mut st = state.lock().unwrap();
                            st.push_raw(headband_id, frame);
                        }
                    }
                    Some(MuseEvent::Connected(name)) => {
                        info!("Headband {headband_id}: Muse BLE connected: {name}");
                        let mut st = state.lock().unwrap();
                        st.status[headband_id] = format!("Connected: {name}");
                        st.connected[headband_id] = true;
                    }
                    Some(MuseEvent::Disconnected) => {
                        warn!("Headband {headband_id}: BLE disconnected");
                        let mut st = state.lock().unwrap();
                        st.status[headband_id] = "Disconnected".to_string();
                        st.connected[headband_id] = false;
                        break;
                    }
                    None => break,
                    _ => {}
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
        st.connected[headband_id] = false;
    }
    Ok(())
}
