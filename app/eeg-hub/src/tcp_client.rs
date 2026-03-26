//! TCP client for streaming EEGM frames to the inference server and
//! receiving reconstructed frames back.
//!
//! Runs two tasks:
//! - **Sender**: reads raw frames from a channel and writes EEGM to the socket
//! - **Receiver**: reads EEGM responses and pushes them into shared state

use std::sync::{Arc, Mutex};

use log::{error, info, warn};
use muse_rs::eegm::EegmFrame;
use tokio::io::{AsyncReadExt, AsyncWriteExt, BufReader, BufWriter};
use tokio::net::TcpStream;
use tokio::sync::mpsc;

use crate::HeadbandState;

/// Outbound frame waiting to be sent to the inference server.
pub type OutboundTx = mpsc::UnboundedSender<EegmFrame>;
pub type OutboundRx = mpsc::UnboundedReceiver<EegmFrame>;

/// Create a send/receive channel pair for outbound EEGM frames.
pub fn outbound_channel() -> (OutboundTx, OutboundRx) {
    mpsc::unbounded_channel()
}

/// Spawn sender + receiver tasks for the inference server connection.
///
/// Returns a `JoinHandle` that completes when the connection drops.
pub fn spawn_tcp_client(
    addr: String,
    state: Arc<Mutex<HeadbandState>>,
    mut outbound_rx: OutboundRx,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        info!("Connecting to inference server at {addr} …");

        let stream = match TcpStream::connect(&addr).await {
            Ok(s) => {
                info!("Connected to inference server");
                {
                    let mut st = state.lock().unwrap();
                    st.server_status = "Connected".to_string();
                    st.server_connected = true;
                }
                s
            }
            Err(e) => {
                error!("Failed to connect to inference server at {addr}: {e}");
                {
                    let mut st = state.lock().unwrap();
                    st.server_status = format!("Connect failed: {e}");
                    st.server_connected = false;
                }
                return;
            }
        };

        let _ = stream.set_nodelay(true);
        let (read_half, write_half) = stream.into_split();
        let mut reader = BufReader::new(read_half);
        let mut writer = BufWriter::new(write_half);

        let state_recv = Arc::clone(&state);

        // Receiver task: read EEGM response frames from server
        let recv_task = tokio::spawn(async move {
            loop {
                match read_eegm_frame(&mut reader).await {
                    Ok(Some(frame)) => {
                        let mut st = state_recv.lock().unwrap();
                        st.push_reconstructed(
                            frame.headband_id as usize,
                            frame,
                        );
                    }
                    Ok(None) => {
                        info!("Inference server closed connection");
                        break;
                    }
                    Err(e) => {
                        error!("Error reading from inference server: {e}");
                        break;
                    }
                }
            }
        });

        // Sender task: forward outbound EEGM frames to server
        let send_task = tokio::spawn(async move {
            while let Some(frame) = outbound_rx.recv().await {
                let encoded = frame.encode();
                if let Err(e) = writer.write_all(&encoded).await {
                    error!("Error writing to inference server: {e}");
                    break;
                }
                if let Err(e) = writer.flush().await {
                    error!("Error flushing to inference server: {e}");
                    break;
                }
            }
        });

        // Wait for either task to finish (connection dropped)
        tokio::select! {
            _ = recv_task => {}
            _ = send_task => {}
        }

        {
            let mut st = state.lock().unwrap();
            st.server_status = "Disconnected".to_string();
            st.server_connected = false;
        }
        warn!("Inference server connection closed");
    })
}

/// Read one EEGM frame from an async reader.
///
/// Returns `Ok(None)` on clean EOF.
async fn read_eegm_frame<R: AsyncReadExt + Unpin>(r: &mut R) -> anyhow::Result<Option<EegmFrame>> {
    use muse_rs::eegm::{HEADER_SIZE, MAGIC_EEGM};

    let mut hdr = [0u8; HEADER_SIZE];
    match r.read_exact(&mut hdr[..4]).await {
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e.into()),
    }
    r.read_exact(&mut hdr[4..]).await?;

    let magic = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
    if magic != MAGIC_EEGM {
        anyhow::bail!("bad EEGM magic: 0x{magic:08X}");
    }

    let headband_id = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
    let epoch_seq = u32::from_le_bytes([hdr[8], hdr[9], hdr[10], hdr[11]]);
    let n_channels = u32::from_le_bytes([hdr[12], hdr[13], hdr[14], hdr[15]]);
    let n_samples = u32::from_le_bytes([hdr[16], hdr[17], hdr[18], hdr[19]]);

    let payload_len = (n_channels as usize) * (n_samples as usize);
    let mut raw = vec![0u8; payload_len * 4];
    r.read_exact(&mut raw).await?;

    let data: Vec<f32> = raw
        .chunks_exact(4)
        .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        .collect();

    Ok(Some(EegmFrame {
        headband_id,
        epoch_seq,
        n_channels,
        n_samples,
        data,
    }))
}
