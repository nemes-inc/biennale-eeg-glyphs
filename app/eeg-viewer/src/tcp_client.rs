//! TCP client for streaming EEGM frames to the inference server and
//! receiving reconstructed frames back.
//!
//! Ported from eeg-hub/src/tcp_client.rs with per-device routing.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use log::{error, info, warn};
use muse_rs::eegm::{
    ConnectAck, ConnectReq, EegmFrame, CTRL_SIZE, HEADER_SIZE, MAGIC_EEGC, MAGIC_EEGM,
};
use tokio::io::{AsyncReadExt, AsyncWriteExt, BufReader, BufWriter};
use tokio::net::TcpStream;

use crate::device_state::{
    extract_channels_from_frame, read_session_frames, DeviceState, OutboundRx,
    ServerState,
};

/// Shared device map — updated by main thread when devices connect/disconnect.
/// TCP recv task reads from this to route reconstructed frames.
pub type DeviceMap = Arc<Mutex<HashMap<u32, Arc<Mutex<DeviceState>>>>>;

/// Session replay info for one device: path + how many frames already sent.
pub struct SessionReplayInfo {
    pub path: PathBuf,
    pub frames_already_sent: u64,
}

/// Spawn TCP client tasks for the inference server.
/// Routes inbound reconstructed frames to correct DeviceState by headband_id.
/// On connect, replays any unsent session frames before streaming live data.
pub fn spawn_tcp_client(
    rt: &tokio::runtime::Runtime,
    addr: String,
    n_headbands: u32,
    sample_rate: u32,
    server_state: Arc<Mutex<ServerState>>,
    device_map: DeviceMap,
    mut outbound_rx: OutboundRx,
    session_replays: Vec<SessionReplayInfo>,
) -> tokio::task::JoinHandle<()> {
    rt.spawn(async move {
        info!("TCP client connecting to {addr}…");
        update_server_status(&server_state, "Connecting…", false);

        let stream = match TcpStream::connect(&addr).await {
            Ok(s) => {
                info!("TCP connected to {addr}");
                s
            }
            Err(e) => {
                error!("Failed to connect to inference server at {addr}: {e}");
                update_server_status(&server_state, &format!("Connect failed: {e}"), false);
                return;
            }
        };

        let _ = stream.set_nodelay(true);
        let (read_half, write_half) = stream.into_split();
        let mut reader = BufReader::new(read_half);
        let mut writer = BufWriter::new(write_half);

        // ── Handshake ──────────────────────────────────────────────────
        update_server_status(&server_state, "Handshake…", false);

        let req = ConnectReq::new(n_headbands, sample_rate);
        let encoded = req.encode();
        info!("Sending ConnectReq ({} bytes): n_headbands={n_headbands}, sample_rate={sample_rate}", encoded.len());
        if let Err(e) = writer.write_all(&encoded).await {
            error!("Failed to send ConnectReq: {e}");
            update_server_status(&server_state, &format!("Handshake failed: {e}"), false);
            return;
        }
        if let Err(e) = writer.flush().await {
            error!("Failed to flush ConnectReq: {e}");
            update_server_status(&server_state, &format!("Handshake failed: {e}"), false);
            return;
        }

        info!("ConnectReq sent, waiting for ConnectAck (10s timeout)…");

        let ack_result = tokio::time::timeout(
            std::time::Duration::from_secs(10),
            read_connect_ack(&mut reader),
        )
        .await;

        match ack_result {
            Ok(Ok(ack)) if ack.is_ok() => {
                info!(
                    "ConnectAck OK — server accepted {} headband(s) @ {} Hz",
                    ack.n_headbands, ack.sample_rate
                );
                update_server_status(
                    &server_state,
                    &format!("Connected ({} band(s), {} Hz)", ack.n_headbands, ack.sample_rate),
                    true,
                );
            }
            Ok(Ok(ack)) => {
                error!("ConnectAck rejected: status={}", ack.status);
                update_server_status(
                    &server_state,
                    &format!("Rejected (status={})", ack.status),
                    false,
                );
                return;
            }
            Ok(Err(e)) => {
                error!("Handshake read error: {e}");
                update_server_status(&server_state, &format!("Handshake failed: {e}"), false);
                return;
            }
            Err(_) => {
                error!("Handshake timed out after 10s — server did not send ConnectAck");
                update_server_status(&server_state, "Handshake timeout (10s)", false);
                return;
            }
        }

        // ── Replay unsent session frames ────────────────────────────────
        for replay in &session_replays {
            match read_session_frames(&replay.path, replay.frames_already_sent) {
                Ok(frames) if !frames.is_empty() => {
                    info!(
                        "Replaying {} unsent frames from {} (offset {})",
                        frames.len(),
                        replay.path.display(),
                        replay.frames_already_sent,
                    );
                    update_server_status(
                        &server_state,
                        &format!("Replaying {} frames…", frames.len()),
                        true,
                    );
                    for frame in &frames {
                        let encoded = frame.encode();
                        if let Err(e) = writer.write_all(&encoded).await {
                            error!("Replay write error: {e}");
                            update_server_status(
                                &server_state,
                                &format!("Replay failed: {e}"),
                                false,
                            );
                            return;
                        }
                    }
                    if let Err(e) = writer.flush().await {
                        error!("Replay flush error: {e}");
                        update_server_status(
                            &server_state,
                            &format!("Replay failed: {e}"),
                            false,
                        );
                        return;
                    }
                    info!("Replay complete for {}", replay.path.display());
                }
                Ok(_) => {} // no unsent frames
                Err(e) => {
                    warn!(
                        "Failed to read session file {} for replay: {e}",
                        replay.path.display()
                    );
                }
            }
        }

        update_server_status(
            &server_state,
            &format!("Connected ({n_headbands} band(s), {sample_rate} Hz)"),
            true,
        );

        // ── Streaming ──────────────────────────────────────────────────
        // device_map is Arc<Mutex<HashMap>> — shared with main thread so
        // devices that connect after the server are still routable.
        let device_map_recv = Arc::clone(&device_map);
        let device_map_send = Arc::clone(&device_map);

        let mut recv_task = tokio::spawn(async move {
            let mut recv_count: u64 = 0;
            loop {
                match read_eegm_message(&mut reader).await {
                    Ok(Some(frame)) => {
                        recv_count += 1;
                        if recv_count <= 3 || recv_count % 100 == 0 {
                            info!(
                                "Recv frame #{recv_count}: hid={} seq={} ch={} samp={}",
                                frame.headband_id, frame.epoch_seq, frame.n_channels, frame.n_samples
                            );
                        }
                        let map = device_map_recv.lock().unwrap();
                        if let Some(state) = map.get(&frame.headband_id) {
                            let channels = extract_channels_from_frame(&frame);
                            state.lock().unwrap().push_reconstructed_frame(channels);
                        } else if recv_count <= 5 {
                            warn!(
                                "No device for headband_id={}, known ids: {:?}",
                                frame.headband_id,
                                map.keys().collect::<Vec<_>>()
                            );
                        }
                    }
                    Ok(None) => {
                        info!("Inference server closed connection (recv_count={recv_count})");
                        break;
                    }
                    Err(e) => {
                        error!("Error reading from inference server: {e} (recv_count={recv_count})");
                        break;
                    }
                }
            }
        });

        let mut send_task = tokio::spawn(async move {
            let mut send_count: u64 = 0;
            info!("TCP send task started, waiting for outbound frames…");
            while let Some(frame) = outbound_rx.recv().await {
                send_count += 1;
                let hid = frame.headband_id;
                if send_count <= 3 || send_count % 100 == 0 {
                    info!("TCP send #{send_count}: hid={hid} seq={}", frame.epoch_seq);
                }
                let encoded = frame.encode();
                if let Err(e) = writer.write_all(&encoded).await {
                    error!("Error writing to inference server: {e}");
                    break;
                }
                if let Err(e) = writer.flush().await {
                    error!("Error flushing to inference server: {e}");
                    break;
                }
                // Track frames sent per device
                let map = device_map_send.lock().unwrap();
                if let Some(state) = map.get(&hid) {
                    state.lock().unwrap().session_frames_sent += 1;
                }
            }
            info!("TCP send task exiting (sent {send_count} total)");
        });

        tokio::select! {
            _ = &mut recv_task => { send_task.abort(); }
            _ = &mut send_task => { recv_task.abort(); }
        }

        update_server_status(&server_state, "Disconnected", false);
        warn!("Inference server connection closed");
    })
}

fn update_server_status(state: &Arc<Mutex<ServerState>>, msg: &str, connected: bool) {
    let mut st = state.lock().unwrap();
    st.status_line = msg.to_string();
    st.connected = connected;
}

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

/// Parse a single EEGM data frame from the stream.
/// Returns `Ok(None)` on clean EOF, `Ok(Some(frame))` on success.
/// Silently skips unexpected EEGC control messages during streaming.
async fn read_eegm_message<R: AsyncReadExt + Unpin>(
    r: &mut R,
) -> anyhow::Result<Option<EegmFrame>> {
    loop {
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

                let headband_id =
                    u32::from_le_bytes([hdr_rest[0], hdr_rest[1], hdr_rest[2], hdr_rest[3]]);
                let epoch_seq =
                    u32::from_le_bytes([hdr_rest[4], hdr_rest[5], hdr_rest[6], hdr_rest[7]]);
                let n_channels =
                    u32::from_le_bytes([hdr_rest[8], hdr_rest[9], hdr_rest[10], hdr_rest[11]]);
                let n_samples =
                    u32::from_le_bytes([hdr_rest[12], hdr_rest[13], hdr_rest[14], hdr_rest[15]]);

                let payload_len = (n_channels as usize) * (n_samples as usize);
                let mut raw = vec![0u8; payload_len * 4];
                r.read_exact(&mut raw).await?;

                let data: Vec<f32> = raw
                    .chunks_exact(4)
                    .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                    .collect();

                return Ok(Some(EegmFrame {
                    headband_id,
                    epoch_seq,
                    n_channels,
                    n_samples,
                    data,
                }));
            }
            MAGIC_EEGC => {
                let mut skip = [0u8; CTRL_SIZE - 4];
                r.read_exact(&mut skip).await?;
                warn!("Skipping unexpected EEGC control message during streaming");
                continue;
            }
            _ => {
                anyhow::bail!("unknown magic: 0x{magic:08X}");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    // ── read_connect_ack ────────────────────────────────────────────────

    #[tokio::test]
    async fn read_connect_ack_ok() {
        let ack = ConnectAck::ok(2, 256);
        let encoded = ack.encode();
        let mut cursor = Cursor::new(encoded);
        let parsed = read_connect_ack(&mut cursor).await.unwrap();
        assert!(parsed.is_ok());
        assert_eq!(parsed.n_headbands, 2);
        assert_eq!(parsed.sample_rate, 256);
    }

    #[tokio::test]
    async fn read_connect_ack_error_status() {
        let ack = ConnectAck::error(42);
        let encoded = ack.encode();
        let mut cursor = Cursor::new(encoded);
        let parsed = read_connect_ack(&mut cursor).await.unwrap();
        assert!(!parsed.is_ok());
        assert_eq!(parsed.status, 42);
    }

    #[tokio::test]
    async fn read_connect_ack_bad_magic() {
        let mut buf = vec![0u8; CTRL_SIZE];
        buf[0..4].copy_from_slice(&0xDEADBEEFu32.to_le_bytes());
        let mut cursor = Cursor::new(buf);
        let err = read_connect_ack(&mut cursor).await.unwrap_err();
        assert!(err.to_string().contains("magic"));
    }

    #[tokio::test]
    async fn read_connect_ack_bad_msg_type() {
        // Valid EEGC magic but wrong message type (1 = ConnectReq instead of 2 = ConnectAck)
        let req = ConnectReq::new(1, 256);
        let encoded = req.encode();
        let mut cursor = Cursor::new(encoded);
        let err = read_connect_ack(&mut cursor).await.unwrap_err();
        assert!(err.to_string().contains("type=1"));
    }

    #[tokio::test]
    async fn read_connect_ack_truncated() {
        let mut cursor = Cursor::new(vec![0u8; 4]); // too short
        assert!(read_connect_ack(&mut cursor).await.is_err());
    }

    // ── read_eegm_message ───────────────────────────────────────────────

    #[tokio::test]
    async fn read_eegm_basic() {
        let channels = vec![vec![1.0f32, 2.0], vec![3.0, 4.0]];
        let frame = EegmFrame::new(0, 7, &channels, 2);
        let encoded = frame.encode();
        let mut cursor = Cursor::new(encoded);
        let parsed = read_eegm_message(&mut cursor).await.unwrap().unwrap();
        assert_eq!(parsed.headband_id, 0);
        assert_eq!(parsed.epoch_seq, 7);
        assert_eq!(parsed.n_channels, 2);
        assert_eq!(parsed.n_samples, 2);
        assert_eq!(parsed.data.len(), 4);
    }

    #[tokio::test]
    async fn read_eegm_eof_returns_none() {
        let mut cursor = Cursor::new(Vec::new());
        let result = read_eegm_message(&mut cursor).await.unwrap();
        assert!(result.is_none());
    }

    #[tokio::test]
    async fn read_eegm_unknown_magic() {
        let bad = 0xDEADBEEFu32.to_le_bytes();
        let mut cursor = Cursor::new(bad.to_vec());
        let err = read_eegm_message(&mut cursor).await.unwrap_err();
        assert!(err.to_string().contains("unknown magic"));
    }

    #[tokio::test]
    async fn read_eegm_skips_eegc_then_reads_eegm() {
        // An EEGC control message followed by a valid EEGM frame
        let ack = ConnectAck::ok(1, 256);
        let channels = vec![vec![1.0f32]; 1];
        let frame = EegmFrame::new(0, 0, &channels, 1);
        let mut buf = ack.encode();
        buf.extend_from_slice(&frame.encode());
        let mut cursor = Cursor::new(buf);
        let parsed = read_eegm_message(&mut cursor).await.unwrap().unwrap();
        assert_eq!(parsed.n_channels, 1);
        assert_eq!(parsed.n_samples, 1);
    }

    #[tokio::test]
    async fn read_eegm_channel_data_roundtrip() {
        let channels = vec![
            vec![1.0f32, 2.0, 3.0],
            vec![4.0, 5.0, 6.0],
            vec![7.0, 8.0, 9.0],
            vec![10.0, 11.0, 12.0],
        ];
        let frame = EegmFrame::new(2, 99, &channels, 3);
        let encoded = frame.encode();
        let mut cursor = Cursor::new(encoded);
        let parsed = read_eegm_message(&mut cursor).await.unwrap().unwrap();
        assert_eq!(parsed.headband_id, 2);
        assert_eq!(parsed.epoch_seq, 99);
        assert_eq!(parsed.n_channels, 4);
        assert_eq!(parsed.n_samples, 3);
        // Verify data integrity via extract_channels_from_frame
        let extracted = extract_channels_from_frame(&parsed);
        assert_eq!(extracted, channels);
    }
}
