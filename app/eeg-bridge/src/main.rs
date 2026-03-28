//! **eeg-bridge** — Single-headband Muse BLE → inference server bridge.
//!
//! Connects to one Muse headband via Bluetooth, displays raw EEG live,
//! streams EEGM frames to the inference server over TCP, and overlays
//! reconstructed results as they arrive (delayed).
//!
//! This is a stopgap until eeg-hub supports real BLE connections.
//!
//! Usage:
//! ```text
//! eeg-bridge --server 192.168.1.50:9100
//! eeg-bridge --server 127.0.0.1:9100          # local echo-mode test
//! eeg-bridge --no-server                       # BLE viewer only
//! ```

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use anyhow::Result;
use clap::Parser;
use eframe::egui;
use egui::RichText;
use egui_plot::{Legend, Line, Plot, PlotPoints};
use log::{error, info, warn};
use muse_rs::eegm::{
    ConnectAck, ConnectReq, EegmFrame, TargetChannelsAck,
    CTRL_SIZE, HEADER_SIZE, MAGIC_EEGC, MAGIC_EEGM,
};
use muse_rs::muse_client::{MuseClient, MuseClientConfig};
use muse_rs::types::MuseEvent;
use tokio::io::{AsyncReadExt, AsyncWriteExt, BufReader, BufWriter};
use tokio::net::TcpStream;
use tokio::sync::mpsc;
use rustfft::{Fft, FftPlanner, num_complex::Complex};

// ── Constants ────────────────────────────────────────────────────────────────

const SAMPLE_RATE: u32 = 256;
const NUM_CHANNELS: usize = 4;
const CHUNK: usize = 256;
const MAX_POINTS: usize = 8192;

// ── FFT / Band Power ─────────────────────────────────────────────────────────

const FFT_WINDOW: usize = 256;      // 1 s @ 256 Hz → 1 Hz resolution
const FFT_HOP: usize = 64;          // new power point every 0.25 s
const NUM_BANDS: usize = 5;
const MAX_BAND_POINTS: usize = 512;  // ~2 min rolling history

const BAND_NAMES: [&str; NUM_BANDS] = ["Delta", "Theta", "Alpha", "Beta", "Gamma"];
const BAND_RANGES: [(f64, f64); NUM_BANDS] = [
    (0.5, 4.0),
    (4.0, 8.0),
    (8.0, 13.0),
    (13.0, 30.0),
    (30.0, 50.0),
];
const BAND_COLORS: [egui::Color32; NUM_BANDS] = [
    egui::Color32::from_rgb(230, 80, 80),   // Delta – red
    egui::Color32::from_rgb(230, 180, 40),  // Theta – amber
    egui::Color32::from_rgb(80, 200, 80),   // Alpha – green
    egui::Color32::from_rgb(80, 140, 255),  // Beta  – blue
    egui::Color32::from_rgb(180, 80, 230),  // Gamma – purple
];

// ── CLI ──────────────────────────────────────────────────────────────────────

#[derive(Parser, Debug)]
#[command(
    name = "eeg-bridge",
    about = "Single-headband Muse BLE → inference server bridge with live overlay"
)]
struct Args {
    /// Inference server address (e.g. 192.168.1.50:9100).
    #[arg(long)]
    server: Option<String>,

    /// Disable inference server connection (BLE viewer only).
    #[arg(long)]
    no_server: bool,

    /// BLE scan timeout in seconds.
    #[arg(long, default_value_t = 30)]
    scan_timeout: u64,

    /// Muse device name prefix for BLE filtering.
    #[arg(long, default_value = "Muse")]
    name_prefix: String,

    /// Rolling buffer size per channel.
    #[arg(long, default_value_t = MAX_POINTS)]
    max_points: usize,

    /// Comma-separated 10-20 channel names for ZUNA upsampling
    /// (e.g. "Fz,Cz,Pz").  Sent to the server after handshake.
    #[arg(long, value_delimiter = ',')]
    target_channels: Option<Vec<String>>,
}

// ── Shared state ─────────────────────────────────────────────────────────────

pub struct BridgeState {
    /// Per-channel rolling buffers of raw BLE samples (µV).
    pub raw: [Vec<f32>; NUM_CHANNELS],
    /// Global sample index of the *next* raw sample to be pushed.
    /// Buffer index `i` maps to global x = `raw_head - raw[0].len() + i`.
    pub raw_head: u64,

    /// Per-channel rolling buffers of reconstructed samples from server (µV).
    pub recon: [Vec<f32>; NUM_CHANNELS],
    /// Global sample index of `recon[0][0]`.
    pub recon_origin: u64,

    /// Counters.
    pub raw_frames: u64,
    pub recon_frames: u64,
    /// Status strings for the toolbar.
    pub ble_status: String,
    pub server_status: String,
    pub ble_connected: bool,
    pub server_connected: bool,
    /// Max rolling buffer size.
    pub max_points: usize,
}

impl BridgeState {
    fn new(max_points: usize) -> Self {
        Self {
            raw: [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
            raw_head: 0,
            recon: [Vec::new(), Vec::new(), Vec::new(), Vec::new()],
            recon_origin: 0,
            raw_frames: 0,
            recon_frames: 0,
            ble_status: "Idle".into(),
            server_status: "Not connected".into(),
            ble_connected: false,
            server_connected: false,
            max_points,
        }
    }

    fn push_raw_chunk(&mut self, channels: &[Vec<f32>; NUM_CHANNELS]) {
        let chunk_len = channels[0].len();
        for ch in 0..NUM_CHANNELS {
            self.raw[ch].extend_from_slice(&channels[ch]);
            let len = self.raw[ch].len();
            if len > self.max_points {
                self.raw[ch].drain(0..len - self.max_points);
            }
        }
        self.raw_head += chunk_len as u64;
        self.raw_frames += 1;

        // Evict stale recon: if raw start has scrolled past recon end, clear it.
        let raw_start = self.raw_head - self.raw[0].len() as u64;
        let recon_end = self.recon_origin + self.recon[0].len() as u64;
        if !self.recon[0].is_empty() && raw_start >= recon_end {
            for ch in 0..NUM_CHANNELS {
                self.recon[ch].clear();
            }
        }
    }

    fn push_recon_frame(&mut self, frame: &EegmFrame) {
        let n_ch = (frame.n_channels as usize).min(NUM_CHANNELS);
        let n_smp = frame.n_samples as usize;
        // Global position of this frame's first sample.
        let frame_start = frame.epoch_seq as u64 * CHUNK as u64;

        if self.recon[0].is_empty() {
            // First recon data ever — set origin.
            self.recon_origin = frame_start;
        }

        let current_end = self.recon_origin + self.recon[0].len() as u64;

        if frame_start + n_smp as u64 <= current_end {
            // Entirely before/within existing data — skip (duplicate/out-of-order).
            return;
        }

        // Fill gap with zeros if the frame doesn't start exactly at current_end.
        if frame_start > current_end {
            let gap = (frame_start - current_end) as usize;
            for ch in 0..NUM_CHANNELS {
                self.recon[ch].extend(std::iter::repeat(0.0f32).take(gap));
            }
        }

        // Append frame data (pad missing channels with zeros).
        for ch in 0..NUM_CHANNELS {
            if ch < n_ch {
                self.recon[ch].extend_from_slice(frame.channel_data(ch));
            } else {
                self.recon[ch].extend(std::iter::repeat(0.0f32).take(n_smp));
            }
        }

        // Trim front if buffer exceeds max_points.
        let len = self.recon[0].len();
        if len > self.max_points {
            let drain = len - self.max_points;
            for ch in 0..NUM_CHANNELS {
                self.recon[ch].drain(0..drain);
            }
            self.recon_origin += drain as u64;
        }

        self.recon_frames += 1;
    }
}

// ── Outbound EEGM channel ───────────────────────────────────────────────────

type OutboundTx = mpsc::UnboundedSender<EegmFrame>;
type OutboundRx = mpsc::UnboundedReceiver<EegmFrame>;

// ── BLE task ─────────────────────────────────────────────────────────────────

async fn run_ble(
    state: Arc<Mutex<BridgeState>>,
    outbound_tx: Option<OutboundTx>,
    name_prefix: String,
    scan_timeout: u64,
) {
    match run_ble_inner(state.clone(), outbound_tx, &name_prefix, scan_timeout).await {
        Ok(()) => info!("BLE loop ended cleanly"),
        Err(e) => {
            error!("BLE error: {e:#}");
            let mut st = state.lock().unwrap();
            st.ble_status = format!("Error: {e}");
            st.ble_connected = false;
        }
    }
}

async fn run_ble_inner(
    state: Arc<Mutex<BridgeState>>,
    outbound_tx: Option<OutboundTx>,
    name_prefix: &str,
    scan_timeout: u64,
) -> Result<()> {
    {
        let mut st = state.lock().unwrap();
        st.ble_status = "Scanning…".into();
        st.ble_connected = false;
    }

    let config = MuseClientConfig {
        enable_aux: false,
        enable_ppg: false,
        scan_timeout_secs: scan_timeout,
        name_prefix: name_prefix.to_string(),
    };

    let client = MuseClient::new(config);
    let (mut rx, handle) = client.connect().await?;
    handle.start(false, false).await?;

    {
        let mut st = state.lock().unwrap();
        st.ble_status = "Connected".into();
        st.ble_connected = true;
    }

    let mut buffers: [Vec<f32>; NUM_CHANNELS] = [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
    let mut epoch_seq: u32 = 0;

    loop {
        tokio::select! {
            evt = rx.recv() => {
                match evt {
                    Some(MuseEvent::Eeg(r)) if r.electrode < NUM_CHANNELS => {
                        buffers[r.electrode].extend(r.samples.iter().map(|&s| s as f32));

                        // When all channels have CHUNK samples, flush
                        while buffers.iter().map(|v| v.len()).min().unwrap_or(0) >= CHUNK {
                            let mut chunk_data: [Vec<f32>; NUM_CHANNELS] =
                                [Vec::new(), Vec::new(), Vec::new(), Vec::new()];
                            for ch in 0..NUM_CHANNELS {
                                chunk_data[ch] = buffers[ch].drain(0..CHUNK).collect();
                            }

                            // Push to display buffer
                            {
                                let mut st = state.lock().unwrap();
                                st.push_raw_chunk(&chunk_data);
                            }

                            // Build EEGM frame and send to inference server
                            if let Some(ref tx) = outbound_tx {
                                let channels_vec: Vec<Vec<f32>> = chunk_data.to_vec();
                                let frame = EegmFrame::new(
                                    0,         // headband_id = 0
                                    epoch_seq,
                                    &channels_vec,
                                    CHUNK,
                                );
                                if tx.send(frame).is_err() {
                                    warn!("Outbound channel closed — inference server disconnected?");
                                }
                                epoch_seq = epoch_seq.wrapping_add(1);
                            }
                        }
                    }
                    Some(MuseEvent::Connected(name)) => {
                        info!("Muse BLE connected: {name}");
                        let mut st = state.lock().unwrap();
                        st.ble_status = format!("Connected: {name}");
                        st.ble_connected = true;
                    }
                    Some(MuseEvent::Disconnected) => {
                        warn!("Muse BLE disconnected");
                        let mut st = state.lock().unwrap();
                        st.ble_status = "Disconnected".into();
                        st.ble_connected = false;
                        break;
                    }
                    None => break,
                    _ => {}
                }
            }
            _ = tokio::time::sleep(Duration::from_millis(50)) => {}
        }
    }

    let _ = handle.disconnect().await;
    {
        let mut st = state.lock().unwrap();
        st.ble_connected = false;
    }
    Ok(())
}

// ── TCP client (inference server) ────────────────────────────────────────────

async fn run_tcp_client(
    addr: String,
    state: Arc<Mutex<BridgeState>>,
    mut outbound_rx: OutboundRx,
    target_channels: Option<Vec<String>>,
) {
    info!("Connecting to inference server at {addr}…");
    {
        let mut st = state.lock().unwrap();
        st.server_status = "Connecting…".into();
    }

    let stream = match TcpStream::connect(&addr).await {
        Ok(s) => s,
        Err(e) => {
            error!("TCP connect failed: {e}");
            let mut st = state.lock().unwrap();
            st.server_status = format!("Connect failed: {e}");
            return;
        }
    };
    let _ = stream.set_nodelay(true);
    let (read_half, write_half) = stream.into_split();
    let mut reader = BufReader::new(read_half);
    let mut writer = BufWriter::new(write_half);

    // ── Handshake ────────────────────────────────────────────────────────
    {
        let mut st = state.lock().unwrap();
        st.server_status = "Handshake…".into();
    }

    let req = ConnectReq::new(1, SAMPLE_RATE);
    if let Err(e) = writer.write_all(&req.encode()).await {
        error!("Failed to send ConnectReq: {e}");
        let mut st = state.lock().unwrap();
        st.server_status = format!("Handshake failed: {e}");
        return;
    }
    let _ = writer.flush().await;
    info!("ConnectReq sent (1 headband, {SAMPLE_RATE} Hz)");

    match read_connect_ack(&mut reader).await {
        Ok(ack) if ack.status == 0 => {
            info!("ConnectAck OK — {} headband(s) @ {} Hz", ack.n_headbands, ack.sample_rate);
            let mut st = state.lock().unwrap();
            st.server_status = format!("Connected ({} Hz)", ack.sample_rate);
            st.server_connected = true;
        }
        Ok(ack) => {
            error!("Server rejected: status={}", ack.status);
            let mut st = state.lock().unwrap();
            st.server_status = format!("Rejected (status={})", ack.status);
            return;
        }
        Err(e) => {
            error!("Handshake failed: {e}");
            let mut st = state.lock().unwrap();
            st.server_status = format!("Handshake failed: {e}");
            return;
        }
    }

    // ── Optional target channels ────────────────────────────────────────
    if let Some(ref channels) = target_channels {
        if !channels.is_empty() {
            use muse_rs::eegm::TargetChannelsReq;
            let tc_req = TargetChannelsReq::new(channels.clone());
            if let Err(e) = writer.write_all(&tc_req.encode()).await {
                error!("Failed to send TargetChannelsReq: {e}");
            } else {
                let _ = writer.flush().await;
                info!("TargetChannelsReq sent: {:?}", channels);
                // Read TargetChannelsAck
                match read_target_channels_ack(&mut reader).await {
                    Ok(ack) if ack.status == 0 => {
                        info!("TargetChannelsAck OK — {} channels", ack.n_target_channels);
                    }
                    Ok(ack) => {
                        warn!("TargetChannelsAck error: status={}", ack.status);
                    }
                    Err(e) => {
                        warn!("Failed to read TargetChannelsAck: {e}");
                    }
                }
            }
        }
    }

    // ── Bidirectional streaming ──────────────────────────────────────────
    let state_recv = Arc::clone(&state);

    let recv_task = tokio::spawn(async move {
        loop {
            match read_eegm_frame(&mut reader).await {
                Ok(Some(frame)) => {
                    info!(
                        "← Reconstructed: seq={} {}ch×{}smp",
                        frame.epoch_seq, frame.n_channels, frame.n_samples,
                    );
                    let mut st = state_recv.lock().unwrap();
                    st.push_recon_frame(&frame);
                }
                Ok(None) => {
                    info!("Server closed connection");
                    break;
                }
                Err(e) => {
                    error!("Read error from server: {e}");
                    break;
                }
            }
        }
    });

    let send_task = tokio::spawn(async move {
        while let Some(frame) = outbound_rx.recv().await {
            let encoded = frame.encode();
            if let Err(e) = writer.write_all(&encoded).await {
                error!("Write error to server: {e}");
                break;
            }
            if let Err(e) = writer.flush().await {
                error!("Flush error to server: {e}");
                break;
            }
        }
    });

    tokio::select! {
        _ = recv_task => {}
        _ = send_task => {}
    }

    {
        let mut st = state.lock().unwrap();
        st.server_status = "Disconnected".into();
        st.server_connected = false;
    }
    warn!("Inference server connection closed");
}

async fn read_connect_ack<R: AsyncReadExt + Unpin>(r: &mut R) -> Result<ConnectAck> {
    let mut buf = [0u8; CTRL_SIZE];
    r.read_exact(&mut buf).await?;

    let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    if magic != MAGIC_EEGC {
        anyhow::bail!("expected EEGC, got 0x{magic:08X}");
    }
    let msg_type = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    if msg_type != 2 {
        anyhow::bail!("expected ConnectAck (type=2), got type={msg_type}");
    }
    Ok(ConnectAck {
        protocol_version: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        n_headbands: u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
        sample_rate: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
        status: u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
    })
}

async fn read_target_channels_ack<R: AsyncReadExt + Unpin>(r: &mut R) -> Result<TargetChannelsAck> {
    let mut buf = [0u8; CTRL_SIZE];
    r.read_exact(&mut buf).await?;

    let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    if magic != MAGIC_EEGC {
        anyhow::bail!("expected EEGC, got 0x{magic:08X}");
    }
    let msg_type = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    if msg_type != 4 {
        anyhow::bail!("expected TargetChannelsAck (type=4), got type={msg_type}");
    }
    Ok(TargetChannelsAck {
        protocol_version: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
        n_target_channels: u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
        status: u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
    })
}

async fn read_eegm_frame<R: AsyncReadExt + Unpin>(r: &mut R) -> Result<Option<EegmFrame>> {
    let mut magic_buf = [0u8; 4];
    match r.read_exact(&mut magic_buf).await {
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e.into()),
    }
    let magic = u32::from_le_bytes(magic_buf);

    match magic {
        MAGIC_EEGM => {
            let mut hdr = [0u8; HEADER_SIZE - 4];
            r.read_exact(&mut hdr).await?;

            let headband_id = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
            let epoch_seq = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
            let n_channels = u32::from_le_bytes([hdr[8], hdr[9], hdr[10], hdr[11]]);
            let n_samples = u32::from_le_bytes([hdr[12], hdr[13], hdr[14], hdr[15]]);
            let timestamp_us = u64::from_le_bytes([
                hdr[16], hdr[17], hdr[18], hdr[19], hdr[20], hdr[21], hdr[22], hdr[23],
            ]);

            let payload_len = (n_channels as usize) * (n_samples as usize);
            let mut raw = vec![0u8; payload_len * 4];
            r.read_exact(&mut raw).await?;

            let data: Vec<f32> = raw
                .chunks_exact(4)
                .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                .collect();

            Ok(Some(EegmFrame { headband_id, epoch_seq, n_channels, n_samples, timestamp_us, data }))
        }
        MAGIC_EEGC => {
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
            Box::pin(read_eegm_frame(r)).await
        }
        _ => anyhow::bail!("unknown magic: 0x{magic:08X}"),
    }
}

// ── FFT band power computation ───────────────────────────────────────────────

fn compute_band_powers(samples: &[f32], fft: &Arc<dyn Fft<f32>>) -> [f64; NUM_BANDS] {
    let n = samples.len();
    debug_assert_eq!(n, FFT_WINDOW);

    // Hann window → complex buffer
    let mut buffer: Vec<Complex<f32>> = samples
        .iter()
        .enumerate()
        .map(|(i, &s)| {
            let w = 0.5
                * (1.0
                    - (2.0 * std::f32::consts::PI * i as f32 / (n as f32 - 1.0)).cos());
            Complex::new(s * w, 0.0)
        })
        .collect();

    fft.process(&mut buffer);

    let freq_res = SAMPLE_RATE as f64 / FFT_WINDOW as f64; // 1 Hz per bin
    let mut powers = [0.0f64; NUM_BANDS];

    for (b, &(lo, hi)) in BAND_RANGES.iter().enumerate() {
        let bin_lo = (lo / freq_res).ceil() as usize;
        let bin_hi = ((hi / freq_res).floor() as usize).min(FFT_WINDOW / 2);
        for k in bin_lo..=bin_hi {
            if k < buffer.len() {
                powers[b] += buffer[k].norm_sqr() as f64;
            }
        }
        // Normalize by window length squared
        powers[b] /= (FFT_WINDOW as f64).powi(2);
    }

    powers
}

// ── Alpha trend tracking ────────────────────────────────────────────────────

const ALPHA_BAND_IDX: usize = 2; // index into BAND_RANGES for 8–13 Hz
const ALPHA_SETTLING_SECS: f64 = 15.0;
const ALPHA_DEADBAND_PCT_PER_MIN: f64 = 2.0;
const MAX_ALPHA_HISTORY: usize = 2400; // ~10 min at 4 Hz hop rate

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AlphaTrendPhase {
    Idle,
    Settling,
    Measuring,
    Complete,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrendLabel {
    Rising,
    Flat,
    Falling,
}

impl std::fmt::Display for TrendLabel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Rising => write!(f, "Rising ↑"),
            Self::Flat => write!(f, "Flat →"),
            Self::Falling => write!(f, "Falling ↓"),
        }
    }
}

/// Online least-squares linear regression accumulator.
#[derive(Debug, Clone, Default)]
struct LinRegAccum {
    n: u64,
    sum_t: f64,
    sum_y: f64,
    sum_tt: f64,
    sum_ty: f64,
    sum_yy: f64,
}

impl LinRegAccum {
    fn reset(&mut self) { *self = Self::default(); }

    fn push(&mut self, t: f64, y: f64) {
        self.n += 1;
        self.sum_t += t;
        self.sum_y += y;
        self.sum_tt += t * t;
        self.sum_ty += t * y;
        self.sum_yy += y * y;
    }

    fn count(&self) -> u64 { self.n }

    fn mean_y(&self) -> f64 {
        if self.n == 0 { 0.0 } else { self.sum_y / self.n as f64 }
    }

    fn slope(&self) -> f64 {
        let n = self.n as f64;
        let denom = n * self.sum_tt - self.sum_t * self.sum_t;
        if denom.abs() < 1e-30 || self.n < 2 { return 0.0; }
        (n * self.sum_ty - self.sum_t * self.sum_y) / denom
    }

    fn intercept(&self) -> f64 {
        if self.n < 2 { return self.mean_y(); }
        (self.sum_y - self.slope() * self.sum_t) / self.n as f64
    }

    fn r_squared(&self) -> f64 {
        let n = self.n as f64;
        if self.n < 3 { return 0.0; }
        let ss_tot = self.sum_yy - self.sum_y * self.sum_y / n;
        if ss_tot.abs() < 1e-30 { return 0.0; }
        let slope = self.slope();
        let intercept = self.intercept();
        let ss_res = self.sum_yy
            - 2.0 * slope * self.sum_ty
            - 2.0 * intercept * self.sum_y
            + slope * slope * self.sum_tt
            + 2.0 * slope * intercept * self.sum_t
            + n * intercept * intercept;
        (1.0 - ss_res / ss_tot).clamp(0.0, 1.0)
    }
}

struct AlphaTrendState {
    phase: AlphaTrendPhase,
    start_time: Option<Instant>,
    reg: LinRegAccum,
    /// (measuring_elapsed_s, alpha_power_linear) pairs for plotting.
    history: Vec<(f64, f64)>,
    /// Cached result fields.
    trend_pct_per_min: f64,
    r_squared: f64,
    label: TrendLabel,
    n_samples: u64,
}

impl AlphaTrendState {
    fn new() -> Self {
        Self {
            phase: AlphaTrendPhase::Idle,
            start_time: None,
            reg: LinRegAccum::default(),
            history: Vec::with_capacity(MAX_ALPHA_HISTORY),
            trend_pct_per_min: 0.0,
            r_squared: 0.0,
            label: TrendLabel::Flat,
            n_samples: 0,
        }
    }

    fn elapsed_s(&self) -> f64 {
        self.start_time.map(|t| t.elapsed().as_secs_f64()).unwrap_or(0.0)
    }

    fn measuring_elapsed_s(&self) -> f64 {
        (self.elapsed_s() - ALPHA_SETTLING_SECS).max(0.0)
    }

    fn start(&mut self) {
        self.phase = AlphaTrendPhase::Settling;
        self.start_time = Some(Instant::now());
        self.reg.reset();
        self.history.clear();
        self.trend_pct_per_min = 0.0;
        self.r_squared = 0.0;
        self.label = TrendLabel::Flat;
        self.n_samples = 0;
    }

    fn stop(&mut self) {
        if matches!(self.phase, AlphaTrendPhase::Settling | AlphaTrendPhase::Measuring) {
            self.update_result();
            self.phase = AlphaTrendPhase::Complete;
        }
    }

    fn toggle(&mut self) {
        match self.phase {
            AlphaTrendPhase::Idle | AlphaTrendPhase::Complete => self.start(),
            AlphaTrendPhase::Settling | AlphaTrendPhase::Measuring => self.stop(),
        }
    }

    /// Feed a new alpha power sample (linear scale, averaged across channels).
    fn feed(&mut self, alpha_linear: f64) {
        let elapsed = self.elapsed_s();

        match self.phase {
            AlphaTrendPhase::Idle | AlphaTrendPhase::Complete => return,
            AlphaTrendPhase::Settling => {
                if elapsed >= ALPHA_SETTLING_SECS {
                    self.phase = AlphaTrendPhase::Measuring;
                } else {
                    return;
                }
            }
            AlphaTrendPhase::Measuring => {}
        }

        let t = self.measuring_elapsed_s();
        self.reg.push(t, alpha_linear);
        self.history.push((t, alpha_linear));
        if self.history.len() > MAX_ALPHA_HISTORY {
            self.history.remove(0);
        }
        self.n_samples = self.reg.count();
        self.update_result();
    }

    fn update_result(&mut self) {
        if self.reg.count() < 2 { return; }
        let slope = self.reg.slope();
        let mean = self.reg.mean_y();
        self.trend_pct_per_min = if mean.abs() > 1e-12 {
            (slope / mean) * 60.0 * 100.0
        } else {
            0.0
        };
        self.r_squared = self.reg.r_squared();
        self.label = if self.trend_pct_per_min > ALPHA_DEADBAND_PCT_PER_MIN {
            TrendLabel::Rising
        } else if self.trend_pct_per_min < -ALPHA_DEADBAND_PCT_PER_MIN {
            TrendLabel::Falling
        } else {
            TrendLabel::Flat
        };
    }
}

// ── egui application ─────────────────────────────────────────────────────────

struct BridgeApp {
    state: Arc<Mutex<BridgeState>>,
    fft_plan: Arc<dyn Fft<f32>>,
    band_powers: [Vec<f64>; NUM_BANDS],
    /// Latest linear (non-dB) alpha power, averaged across channels.
    latest_alpha_linear: f64,
    fft_cursor: u64,
    alpha_trend: AlphaTrendState,
}

impl BridgeApp {
    fn new(state: Arc<Mutex<BridgeState>>) -> Self {
        let mut planner = FftPlanner::<f32>::new();
        let fft_plan = planner.plan_fft_forward(FFT_WINDOW);
        Self {
            state,
            fft_plan,
            band_powers: [Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new()],
            latest_alpha_linear: 0.0,
            fft_cursor: 0,
            alpha_trend: AlphaTrendState::new(),
        }
    }
}

impl eframe::App for BridgeApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after(Duration::from_millis(33)); // ~30 fps

        let (
            recon_snap, recon_origin,
            ble_status, server_status, ble_ok, srv_ok, raw_n, recon_n,
        ) = {
            let st = self.state.lock().unwrap();
            (
                st.recon.clone(),
                st.recon_origin,
                st.ble_status.clone(),
                st.server_status.clone(),
                st.ble_connected,
                st.server_connected,
                st.raw_frames,
                st.recon_frames,
            )
        };

        // ── Compute band powers on new recon data ────────────────────────
        let recon_len = recon_snap[0].len();
        let recon_end = recon_origin + recon_len as u64;

        // Reset cursor if recon buffer was cleared / rewound
        if self.fft_cursor < recon_origin {
            self.fft_cursor = recon_origin;
        }

        while self.fft_cursor + FFT_WINDOW as u64 <= recon_end {
            let win_start = (self.fft_cursor - recon_origin) as usize;
            let win_end = win_start + FFT_WINDOW;

            if win_end <= recon_len {
                let mut avg = [0.0f64; NUM_BANDS];
                for ch in 0..NUM_CHANNELS {
                    let p = compute_band_powers(
                        &recon_snap[ch][win_start..win_end],
                        &self.fft_plan,
                    );
                    for b in 0..NUM_BANDS {
                        avg[b] += p[b];
                    }
                }
                for b in 0..NUM_BANDS {
                    avg[b] /= NUM_CHANNELS as f64;
                    let db = 10.0 * avg[b].max(1e-20).log10();
                    self.band_powers[b].push(db);
                    if self.band_powers[b].len() > MAX_BAND_POINTS {
                        self.band_powers[b].remove(0);
                    }
                }
                // Feed alpha power (linear scale) to trend tracker
                self.latest_alpha_linear = avg[ALPHA_BAND_IDX];
                self.alpha_trend.feed(self.latest_alpha_linear);
            }
            self.fft_cursor += FFT_HOP as u64;
        }

        // ── Handle 'A' key for alpha trend toggle ────────────────────────
        if ctx.input(|i| i.key_pressed(egui::Key::A)) {
            self.alpha_trend.toggle();
        }

        // ── Toolbar ──────────────────────────────────────────────────────
        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(RichText::new("EEG Bridge").strong());
                ui.separator();

                let ble_color = if ble_ok {
                    egui::Color32::from_rgb(80, 200, 120)
                } else {
                    egui::Color32::GRAY
                };
                ui.colored_label(ble_color, format!("● BLE: {ble_status}"));
                ui.separator();

                let srv_color = if srv_ok {
                    egui::Color32::from_rgb(80, 200, 120)
                } else {
                    egui::Color32::GRAY
                };
                ui.colored_label(srv_color, format!("● Server: {server_status}"));
                ui.separator();

                ui.label(format!("raw: {raw_n}  recon: {recon_n}"));
            });
        });

        // ── Alpha trend panel (bottom) ────────────────────────────────────
        egui::TopBottomPanel::bottom("alpha_trend_panel")
            .resizable(true)
            .default_height(180.0)
            .show(ctx, |ui| {
                self.draw_alpha_trend_panel(ui);
            });

        // ── Band-power plot ───────────────────────────────────────────────
        egui::CentralPanel::default().show(ctx, |ui| {
            if self.band_powers[0].is_empty() {
                ui.centered_and_justified(|ui| {
                    ui.label(
                        RichText::new("Waiting for processed data from inference server…")
                            .size(18.0)
                            .weak(),
                    );
                });
                return;
            }

            let plot = Plot::new("band_power_plot")
                .legend(Legend::default())
                .show_axes(true)
                .allow_scroll(false);

            plot.show(ui, |plot_ui| {
                for b in 0..NUM_BANDS {
                    let points: PlotPoints = self.band_powers[b]
                        .iter()
                        .enumerate()
                        .map(|(i, &v)| [i as f64, v])
                        .collect();
                    plot_ui.line(
                        Line::new(points)
                            .name(BAND_NAMES[b])
                            .color(BAND_COLORS[b])
                            .width(2.0),
                    );
                }
            });
        });
    }
}

impl BridgeApp {
    fn draw_alpha_trend_panel(&self, ui: &mut egui::Ui) {
        let at = &self.alpha_trend;

        ui.horizontal(|ui| {
            ui.label(RichText::new("Alpha Trend").strong().size(14.0));
            ui.separator();

            match at.phase {
                AlphaTrendPhase::Idle => {
                    ui.colored_label(egui::Color32::GRAY, "Idle — press [A] to start");
                }
                AlphaTrendPhase::Settling => {
                    let remaining = (ALPHA_SETTLING_SECS - at.elapsed_s()).max(0.0);
                    ui.colored_label(
                        egui::Color32::from_rgb(230, 180, 40),
                        format!("Settling… {remaining:.0}s remaining"),
                    );
                }
                AlphaTrendPhase::Measuring => {
                    let dur = at.measuring_elapsed_s();
                    ui.colored_label(
                        egui::Color32::from_rgb(80, 200, 80),
                        format!("Measuring — {dur:.0}s"),
                    );
                    ui.separator();
                    let label_color = match at.label {
                        TrendLabel::Rising => egui::Color32::from_rgb(80, 200, 80),
                        TrendLabel::Flat => egui::Color32::from_rgb(180, 180, 180),
                        TrendLabel::Falling => egui::Color32::from_rgb(230, 80, 80),
                    };
                    ui.colored_label(
                        label_color,
                        format!(
                            "{} ({:+.1}%/min)  R²={:.2}  n={}",
                            at.label, at.trend_pct_per_min, at.r_squared, at.n_samples,
                        ),
                    );
                    ui.separator();
                    ui.colored_label(egui::Color32::GRAY, "press [A] to stop");
                }
                AlphaTrendPhase::Complete => {
                    let dur = at.measuring_elapsed_s();
                    let label_color = match at.label {
                        TrendLabel::Rising => egui::Color32::from_rgb(80, 200, 80),
                        TrendLabel::Flat => egui::Color32::from_rgb(180, 180, 180),
                        TrendLabel::Falling => egui::Color32::from_rgb(230, 80, 80),
                    };
                    ui.colored_label(
                        label_color,
                        RichText::new(format!(
                            "RESULT: {} ({:+.1}%/min)  R²={:.2}  n={}  duration={dur:.0}s",
                            at.label, at.trend_pct_per_min, at.r_squared, at.n_samples,
                        ))
                        .strong(),
                    );
                    ui.separator();
                    ui.colored_label(egui::Color32::GRAY, "press [A] to restart");
                }
            }
        });

        // Alpha history plot (only when we have data)
        if !at.history.is_empty() {
            let plot = Plot::new("alpha_trend_plot")
                .show_axes(true)
                .allow_scroll(false)
                .allow_drag(false)
                .allow_zoom(false)
                .y_axis_label("α power")
                .x_axis_label("time (s)")
                .height(ui.available_height().max(60.0));

            plot.show(ui, |plot_ui| {
                // Alpha power data points
                let alpha_points: PlotPoints = at
                    .history
                    .iter()
                    .map(|&(t, v)| [t, v])
                    .collect();
                plot_ui.line(
                    Line::new(alpha_points)
                        .name("Alpha power")
                        .color(egui::Color32::from_rgb(80, 200, 80))
                        .width(1.5),
                );

                // Regression line (if enough points)
                if at.reg.count() >= 2 && !at.history.is_empty() {
                    let t_min = at.history.first().map(|h| h.0).unwrap_or(0.0);
                    let t_max = at.history.last().map(|h| h.0).unwrap_or(1.0);
                    let slope = at.reg.slope();
                    let intercept = at.reg.intercept();
                    let reg_points: PlotPoints = [t_min, t_max]
                        .iter()
                        .map(|&t| [t, slope * t + intercept])
                        .collect();
                    plot_ui.line(
                        Line::new(reg_points)
                            .name("Trend")
                            .color(egui::Color32::from_rgb(255, 120, 60))
                            .width(2.0)
                            .style(egui_plot::LineStyle::Dashed { length: 8.0 }),
                    );
                }
            });
        }
    }
}

// ── Main ─────────────────────────────────────────────────────────────────────

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = Args::parse();
    let state = Arc::new(Mutex::new(BridgeState::new(args.max_points)));

    // Tokio runtime for BLE + TCP
    let rt = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .expect("tokio runtime");

    // Start TCP client (if --server specified)
    let outbound_tx = if !args.no_server {
        if let Some(ref addr) = args.server {
            let (tx, rx) = mpsc::unbounded_channel();
            let st = Arc::clone(&state);
            let addr = addr.clone();
            let tc = args.target_channels.clone();
            rt.spawn(async move {
                run_tcp_client(addr, st, rx, tc).await;
            });
            Some(tx)
        } else {
            log::warn!("No --server specified and --no-server not set; running without inference");
            None
        }
    } else {
        None
    };

    // Start BLE task
    {
        let st = Arc::clone(&state);
        let prefix = args.name_prefix.clone();
        let timeout = args.scan_timeout;
        let tx = outbound_tx.clone();
        rt.spawn(async move {
            run_ble(st, tx, prefix, timeout).await;
        });
    }

    // egui window
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 700.0])
            .with_title("EEG Bridge — Muse BLE → Inference Server"),
        ..Default::default()
    };

    let state_ui = Arc::clone(&state);
    eframe::run_native(
        "EEG Bridge",
        native_options,
        Box::new(move |_cc| Ok(Box::new(BridgeApp::new(state_ui)))),
    )
    .expect("eframe");
}
