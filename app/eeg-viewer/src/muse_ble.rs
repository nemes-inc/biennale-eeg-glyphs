//! In-process Muse Bluetooth → rolling plot buffers (no TCP hop).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use anyhow::Result;
use muse_rs::muse_client::{MuseClient, MuseClientConfig};
use muse_rs::types::MuseEvent;

use crate::{merge_single, SharedState};

const CHUNK: usize = 256;

/// Scan BLE handshake timeouts from `muse_rs::MuseClient`
fn muse_connect_failure_is_retryable(formatted: &str) -> bool {
    let s = formatted.to_ascii_lowercase();
    s.contains("timed out scanning")
        || s.contains("ble connect() timed out")
        || s.contains("discover_services() timed out")
}

pub struct MuseLiveHandle {
    pub stop: Arc<AtomicBool>,
    join: JoinHandle<()>,
}

impl MuseLiveHandle {
    pub fn stop_and_join(self) {
        self.stop.store(true, Ordering::SeqCst);
        let _ = self.join.join();
    }
}

pub fn spawn_muse_ble_thread(
    state: Arc<Mutex<SharedState>>,
    max_points: usize,
) -> MuseLiveHandle {
    let stop = Arc::new(AtomicBool::new(false));
    let stop_clone = Arc::clone(&stop);
    let state_err = Arc::clone(&state);
    let join = thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("tokio runtime for Muse BLE");
        rt.block_on(async move {
            match run_muse_ble_loop(state, max_points, stop_clone).await {
                Ok(()) => {}
                Err(e) => {
                    let msg = format!("Muse BLE: {e:#}");
                    let retryable = muse_connect_failure_is_retryable(&msg);
                    let mut st = state_err.lock().unwrap();
                    st.last_error = Some(msg);
                    st.muse_ble_streaming = false;
                    st.muse_ble_retry_pending = retryable;
                }
            }
        });
    });
    MuseLiveHandle { stop, join }
}

async fn run_muse_ble_loop(
    state: Arc<Mutex<SharedState>>,
    max_points: usize,
    stop: Arc<AtomicBool>,
) -> Result<()> {
    {
        let mut st = state.lock().unwrap();
        st.last_error = None;
        st.muse_ble_streaming = false;
        st.muse_ble_retry_pending = false;
    }

    let config = MuseClientConfig {
        enable_aux: false,
        enable_ppg: false,
        scan_timeout_secs: 15,
        name_prefix: "Muse".to_string(),
    };
    let client = MuseClient::new(config);
    let (mut rx, handle) = client.connect().await?;
    handle.start(false, false).await?;

    {
        let mut st = state.lock().unwrap();
        st.muse_ble_streaming = true;
        st.last_error = None;
        st.muse_ble_retry_pending = false;
    }

    let mut buffers: Vec<Vec<f32>> = vec![Vec::new(); 4];

    loop {
        tokio::select! {
            evt = rx.recv() => {
                if stop.load(Ordering::SeqCst) {
                    break;
                }
                match evt {
                    Some(evt) => match evt {
                        MuseEvent::Eeg(r) if r.electrode < 4 => {
                            buffers[r.electrode]
                                .extend(r.samples.iter().map(|&s| s as f32));
                            while buffers.iter().map(|v| v.len()).min().unwrap_or(0) >= CHUNK {
                                if stop.load(Ordering::SeqCst) {
                                    break;
                                }
                                let frame: Vec<Vec<f32>> = buffers
                                    .iter_mut()
                                    .map(|b| b.drain(0..CHUNK).collect())
                                    .collect();
                                let mut st = state.lock().unwrap();
                                st.last_error = None;
                                merge_single(&mut st, frame, max_points);
                            }
                        }
                        MuseEvent::Disconnected => break,
                        _ => {}
                    },
                    None => break,
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
        st.muse_ble_streaming = false;
    }
    Ok(())
}
