//! TCP client for streaming EEGM frames to the inference server and
//! receiving reconstructed frames back.
//!
//! Connection lifecycle:
//! 1. TCP connect
//! 2. Send `ConnectReq` (n_headbands, sample_rate)
//! 3. Wait for `ConnectAck` — if `status != 0`, abort
//! 4. Spawn sender + receiver tasks for bidirectional streaming

use std::sync::{Arc, Mutex};

use log::{error, info, warn};
use muse_rs::eegm::{
    ConnectAck, ConnectReq, EegmFrame,
    CTRL_SIZE, HEADER_SIZE, MAGIC_EEGC, MAGIC_EEGM,
};
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
/// Performs a ConnectReq/ConnectAck handshake before streaming.
/// Returns a `JoinHandle` that completes when the connection drops.
pub fn spawn_tcp_client(
    addr: String,
    n_headbands: u32,
    sample_rate: u32,
    state: Arc<Mutex<HeadbandState>>,
    mut outbound_rx: OutboundRx,
) -> tokio::task::JoinHandle<()> {
    tokio::spawn(async move {
        info!("Connecting to inference server at {addr} …");
        {
            let mut st = state.lock().unwrap();
            st.server_status = "Connecting…".to_string();
        }

        let stream = match TcpStream::connect(&addr).await {
            Ok(s) => s,
            Err(e) => {
                error!("Failed to connect to inference server at {addr}: {e}");
                let mut st = state.lock().unwrap();
                st.server_status = format!("Connect failed: {e}");
                st.server_connected = false;
                return;
            }
        };

        let _ = stream.set_nodelay(true);
        let (read_half, write_half) = stream.into_split();
        let mut reader = BufReader::new(read_half);
        let mut writer = BufWriter::new(write_half);

        // ── Handshake ────────────────────────────────────────────────────
        {
            let mut st = state.lock().unwrap();
            st.server_status = "Handshake…".to_string();
        }

        let req = ConnectReq::new(n_headbands, sample_rate);
        if let Err(e) = writer.write_all(&req.encode()).await {
            error!("Failed to send ConnectReq: {e}");
            let mut st = state.lock().unwrap();
            st.server_status = format!("Handshake failed: {e}");
            return;
        }
        if let Err(e) = writer.flush().await {
            error!("Failed to flush ConnectReq: {e}");
            let mut st = state.lock().unwrap();
            st.server_status = format!("Handshake failed: {e}");
            return;
        }

        info!("ConnectReq sent ({n_headbands} headband(s), {sample_rate} Hz). Waiting for ack…");

        match read_connect_ack(&mut reader).await {
            Ok(ack) if ack.is_ok() => {
                info!(
                    "ConnectAck OK — server accepted {} headband(s) @ {} Hz",
                    ack.n_headbands, ack.sample_rate
                );
                let mut st = state.lock().unwrap();
                st.server_status = format!(
                    "Connected ({} band(s), {} Hz)",
                    ack.n_headbands, ack.sample_rate
                );
                st.server_connected = true;
            }
            Ok(ack) => {
                error!("ConnectAck rejected: status={}", ack.status);
                let mut st = state.lock().unwrap();
                st.server_status = format!("Rejected (status={})", ack.status);
                st.server_connected = false;
                return;
            }
            Err(e) => {
                error!("Handshake failed: {e}");
                let mut st = state.lock().unwrap();
                st.server_status = format!("Handshake failed: {e}");
                st.server_connected = false;
                return;
            }
        }

        // ── Streaming ────────────────────────────────────────────────────
        let state_recv = Arc::clone(&state);

        // Receiver task: read EEGM response frames from server
        let recv_task = tokio::spawn(async move {
            loop {
                match read_eegm_message(&mut reader).await {
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

/// Read a ConnectAck from the server (expected immediately after sending ConnectReq).
async fn read_connect_ack<R: AsyncReadExt + Unpin>(r: &mut R) -> anyhow::Result<ConnectAck> {
    let mut buf = [0u8; CTRL_SIZE];
    r.read_exact(&mut buf).await?;

    let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    if magic != MAGIC_EEGC {
        anyhow::bail!("expected EEGC ConnectAck, got magic 0x{magic:08X}");
    }
    let msg_type = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    if msg_type != 2 {
        anyhow::bail!("expected ConnectAck (type=2), got type={msg_type}");
    }
    let protocol_version = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
    let n_headbands = u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);
    let sample_rate = u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]);
    let status = u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]);

    Ok(ConnectAck {
        protocol_version,
        n_headbands,
        sample_rate,
        status,
    })
}

/// Read the next EEGM data frame from the server, skipping any unexpected
/// control messages that arrive during streaming.
///
/// Returns `Ok(None)` on clean EOF.
async fn read_eegm_message<R: AsyncReadExt + Unpin>(r: &mut R) -> anyhow::Result<Option<EegmFrame>> {
    let mut magic_buf = [0u8; 4];
    match r.read_exact(&mut magic_buf).await {
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e.into()),
    }
    let magic = u32::from_le_bytes(magic_buf);

    match magic {
        MAGIC_EEGM => {
            let mut hdr_rest = [0u8; HEADER_SIZE - 4];
            r.read_exact(&mut hdr_rest).await?;

            let headband_id = u32::from_le_bytes([hdr_rest[0], hdr_rest[1], hdr_rest[2], hdr_rest[3]]);
            let epoch_seq = u32::from_le_bytes([hdr_rest[4], hdr_rest[5], hdr_rest[6], hdr_rest[7]]);
            let n_channels = u32::from_le_bytes([hdr_rest[8], hdr_rest[9], hdr_rest[10], hdr_rest[11]]);
            let n_samples = u32::from_le_bytes([hdr_rest[12], hdr_rest[13], hdr_rest[14], hdr_rest[15]]);
            let timestamp_us = u64::from_le_bytes([
                hdr_rest[16], hdr_rest[17], hdr_rest[18], hdr_rest[19],
                hdr_rest[20], hdr_rest[21], hdr_rest[22], hdr_rest[23],
            ]);

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
                timestamp_us,
                data,
            }))
        }
        MAGIC_EEGC => {
            // Skip unexpected control message during streaming
            let mut skip = [0u8; CTRL_SIZE - 4];
            r.read_exact(&mut skip).await?;
            // If this is a TargetChannelsReq (msg_type=3), drain the
            // variable-length payload so we don't corrupt the stream.
            let msg_type = u32::from_le_bytes([skip[0], skip[1], skip[2], skip[3]]);
            if msg_type == 3 {
                let payload_len = u32::from_le_bytes([skip[12], skip[13], skip[14], skip[15]]) as usize;
                let mut payload = vec![0u8; payload_len];
                r.read_exact(&mut payload).await?;
            }
            warn!("Skipping unexpected EEGC control message during streaming");
            // Recurse to get next data frame
            Box::pin(read_eegm_message(r)).await
        }
        _ => {
            anyhow::bail!("unknown magic: 0x{magic:08X}");
        }
    }
}
