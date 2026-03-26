//! Per-device EEG state, brand types, and pure semantic functions.

use std::collections::HashSet;
use std::fs::{self, File, OpenOptions};
use std::io::{self, Read as _, Seek, SeekFrom, Write as _};
use std::path::{Path, PathBuf};

use std::sync::Arc;

use muse_rs::eegm::{EegmFrame, HEADER_SIZE};
use tokio::sync::mpsc;

/// Channel for sending EEGM frames to the inference server.
pub type OutboundTx = mpsc::UnboundedSender<EegmFrame>;
pub type OutboundRx = mpsc::UnboundedReceiver<EegmFrame>;

/// Shared outbound sender that BLE tasks hold. The inner sender can be
/// swapped when the server reconnects (new channel, new rx).
#[derive(Clone)]
pub struct SharedOutboundTx(Arc<std::sync::Mutex<Option<OutboundTx>>>);

impl SharedOutboundTx {
    pub fn new() -> Self {
        Self(Arc::new(std::sync::Mutex::new(None)))
    }

    /// Replace the inner sender (called when creating a new server connection).
    /// Returns the new OutboundRx to hand to the TCP client.
    pub fn reset_channel(&self) -> OutboundRx {
        let (tx, rx) = mpsc::unbounded_channel();
        *self.0.lock().unwrap() = Some(tx);
        rx
    }

    /// Try to send a frame. Returns false if no channel is active.
    pub fn send(&self, frame: EegmFrame) -> bool {
        if let Some(ref tx) = *self.0.lock().unwrap() {
            tx.send(frame).is_ok()
        } else {
            false
        }
    }
}


// ── Brand types ──────────────────────────────────────────────────────────────

/// Headband slot index (0–3). Determines EEGM protocol `headband_id`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct HeadbandId(u32);

impl HeadbandId {
    pub const MAX: u32 = 4;

    pub fn new(id: u32) -> Option<Self> {
        (id < Self::MAX).then_some(Self(id))
    }

    pub fn as_u32(self) -> u32 {
        self.0
    }
}

// ── DeviceState ──────────────────────────────────────────────────────────────

/// Muse EEG sample rate in Hz.
const SAMPLE_RATE: f64 = 256.0;

/// Per-device EEG streaming state.
/// Accessed by: BLE task (writer), UI thread (reader), recording save thread (drainer).
#[allow(dead_code)]
pub struct DeviceState {
    pub device_name: String,
    pub device_id: String,

    /// Rolling plot buffers: [channel][[time_secs, value]], trimmed to max_points.
    /// Time is relative to stream start (first raw frame).
    pub raw_channels: Vec<Vec<[f64; 2]>>,
    /// Reconstructed channels from inference server, same time axis.
    pub recon_channels: Vec<Vec<[f64; 2]>>,
    pub raw_frame_count: u64,
    pub recon_frame_count: u64,

    /// True while BLE task is delivering data.
    pub streaming: bool,
    pub status_line: String,

    /// True while accumulating samples into record_buffer.
    pub recording_active: bool,
    /// Unbounded accumulation buffer: [channel][all_samples] (plain f32 for CSV export).
    pub record_buffer: Vec<Vec<f32>>,
    pub record_sample_count: usize,

    /// Frames written to session file on disk.
    pub session_frames_written: u64,
    /// Frames sent to inference server.
    pub session_frames_sent: u64,

    /// Timestamp of the first raw frame (microseconds since epoch).
    /// Used to compute relative time for plot x-axis.
    stream_start_us: Option<u64>,

    max_points: usize,
}

impl DeviceState {
    pub fn new(device_name: String, device_id: String, max_points: usize) -> Self {
        Self {
            device_name,
            device_id,
            raw_channels: vec![Vec::new(); 4],
            recon_channels: Vec::new(),
            raw_frame_count: 0,
            recon_frame_count: 0,
            streaming: false,
            status_line: "Initializing…".to_string(),
            recording_active: false,
            record_buffer: Vec::new(),
            record_sample_count: 0,
            session_frames_written: 0,
            session_frames_sent: 0,
            stream_start_us: None,
            max_points,
        }
    }

    /// Push a raw EEG frame (one CHUNK of aligned channel data) with its capture timestamp.
    pub fn push_raw_frame(&mut self, channels: Vec<Vec<f32>>, timestamp_us: u64) {
        if self.stream_start_us.is_none() && timestamp_us > 0 {
            self.stream_start_us = Some(timestamp_us);
        }
        let base_secs = self.relative_secs(timestamp_us);

        for (ch_idx, samples) in channels.iter().enumerate() {
            if ch_idx >= self.raw_channels.len() {
                break;
            }
            let timed = samples_to_timed(samples, base_secs);
            push_rolling_timed(&mut self.raw_channels[ch_idx], &timed, self.max_points);

            if self.recording_active {
                if self.record_buffer.len() <= ch_idx {
                    self.record_buffer
                        .resize_with(ch_idx + 1, Vec::new);
                }
                push_accumulating(&mut self.record_buffer[ch_idx], samples);
            }
        }
        if self.recording_active {
            self.record_sample_count = self
                .record_buffer
                .iter()
                .map(|c| c.len())
                .min()
                .unwrap_or(0);
        }
        self.raw_frame_count += 1;
    }

    /// Push reconstructed EEG from inference server with the original capture timestamp.
    /// If `timestamp_us` is 0 (server didn't preserve it), falls back to current raw position.
    pub fn push_reconstructed_frame(&mut self, channels: Vec<Vec<f32>>, timestamp_us: u64) {
        let base_secs = if timestamp_us > 0 {
            self.relative_secs(timestamp_us)
        } else {
            // Fallback: place at current end of raw timeline
            self.raw_channels.first()
                .and_then(|ch| ch.last())
                .map(|pt| pt[0])
                .unwrap_or(0.0)
        };

        while self.recon_channels.len() < channels.len() {
            self.recon_channels.push(Vec::new());
        }
        for (ch_idx, samples) in channels.iter().enumerate() {
            let timed = samples_to_timed(samples, base_secs);
            push_rolling_timed(&mut self.recon_channels[ch_idx], &timed, self.max_points);
        }
        self.recon_frame_count += 1;
    }

    /// Convert an absolute timestamp to seconds relative to stream start.
    fn relative_secs(&self, timestamp_us: u64) -> f64 {
        match self.stream_start_us {
            Some(start) => timestamp_us.saturating_sub(start) as f64 / 1_000_000.0,
            None => 0.0,
        }
    }

    /// Start recording: clear buffer, set flag.
    pub fn start_recording(&mut self) {
        self.record_buffer = vec![Vec::new(); self.raw_channels.len()];
        self.record_sample_count = 0;
        self.recording_active = true;
    }

    /// Take recording buffer, leaving empty vecs.
    pub fn take_recording(&mut self) -> (Vec<Vec<f32>>, usize) {
        self.recording_active = false;
        let buf = std::mem::take(&mut self.record_buffer);
        let count = self.record_sample_count;
        self.record_sample_count = 0;
        (buf, count)
    }

    /// Cancel recording without saving.
    #[allow(dead_code)]
    pub fn cancel_recording(&mut self) {
        self.recording_active = false;
        self.record_buffer.clear();
        self.record_sample_count = 0;
    }
}

// ── ServerState ──────────────────────────────────────────────────────────────

/// TCP connection state for the inference server.
pub struct ServerState {
    pub connected: bool,
    pub status_line: String,
}

impl Default for ServerState {
    fn default() -> Self {
        Self {
            connected: false,
            status_line: "Not connected".to_string(),
        }
    }
}

// ── SessionWriter ────────────────────────────────────────────────────────────

/// Generate session file path: `{base_dir}/sessions/{safe_name}_{timestamp}.eegm`
pub fn session_file_path(base_dir: &Path, device_name: &str) -> PathBuf {
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0);
    let safe_name = device_name.replace(|c: char| !c.is_alphanumeric() && c != '-', "_");
    base_dir
        .join("sessions")
        .join(format!("{safe_name}_{ts}.eegm"))
}

/// Appends encoded EEGM frames to a binary session file on disk.
/// Tracks how many frames have been written and how many have been sent
/// to the inference server, enabling replay of unsent frames on reconnect.
///
/// Session file format: concatenated raw EEGM binary frames (self-describing
/// via 28-byte headers: magic + headband_id + epoch_seq + n_channels + n_samples + timestamp_us).
pub struct SessionWriter {
    file: File,
    /// Number of frames appended.
    frames_written: u64,
}

impl SessionWriter {
    /// Create a new session file. Creates parent directories if needed.
    pub fn create(path: &Path) -> io::Result<Self> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        let file = File::create(path)?;
        Ok(Self {
            file,
            frames_written: 0,
        })
    }

    /// Append an encoded EEGM frame to the session file.
    pub fn append_frame(&mut self, frame: &EegmFrame) -> io::Result<()> {
        let encoded = frame.encode();
        self.file.write_all(&encoded)?;
        self.file.flush()?;
        self.frames_written += 1;
        Ok(())
    }

    /// Total frames written to disk.
    pub fn frames_written(&self) -> u64 {
        self.frames_written
    }
}

/// Read all EEGM frames from a session file starting at frame index `from_frame`.
/// Returns the frames in order. Used for replay on server reconnect.
pub fn read_session_frames(path: &Path, from_frame: u64) -> io::Result<Vec<EegmFrame>> {
    let mut file = OpenOptions::new().read(true).open(path)?;
    let mut frames = Vec::new();
    let mut frame_index: u64 = 0;

    loop {
        // Read 4-byte magic
        let mut magic_buf = [0u8; 4];
        match file.read_exact(&mut magic_buf) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => break,
            Err(e) => return Err(e),
        }

        // Read rest of header (HEADER_SIZE - 4 bytes)
        let mut hdr_rest = [0u8; HEADER_SIZE - 4];
        file.read_exact(&mut hdr_rest)?;

        let n_channels = u32::from_le_bytes([hdr_rest[8], hdr_rest[9], hdr_rest[10], hdr_rest[11]]);
        let n_samples = u32::from_le_bytes([hdr_rest[12], hdr_rest[13], hdr_rest[14], hdr_rest[15]]);
        let payload_len = (n_channels as usize) * (n_samples as usize) * 4;

        if frame_index < from_frame {
            // Skip the payload
            file.seek(SeekFrom::Current(payload_len as i64))?;
            frame_index += 1;
            continue;
        }

        // Read payload
        let mut raw = vec![0u8; payload_len];
        file.read_exact(&mut raw)?;

        let headband_id = u32::from_le_bytes([hdr_rest[0], hdr_rest[1], hdr_rest[2], hdr_rest[3]]);
        let epoch_seq = u32::from_le_bytes([hdr_rest[4], hdr_rest[5], hdr_rest[6], hdr_rest[7]]);
        let timestamp_us = u64::from_le_bytes([
            hdr_rest[16], hdr_rest[17], hdr_rest[18], hdr_rest[19],
            hdr_rest[20], hdr_rest[21], hdr_rest[22], hdr_rest[23],
        ]);
        let data: Vec<f32> = raw
            .chunks_exact(4)
            .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .collect();

        frames.push(EegmFrame {
            headband_id,
            epoch_seq,
            n_channels,
            n_samples,
            timestamp_us,
            data,
        });
        frame_index += 1;
    }

    Ok(frames)
}

// ── ConnectedDevice ──────────────────────────────────────────────────────────

/// Ownership bundle for one active connection (UI-thread only fields).
#[allow(dead_code)]
pub struct ConnectedDevice {
    pub device_name: String,
    pub device_id: String,
    pub headband_id: HeadbandId,
    pub state: std::sync::Arc<std::sync::Mutex<DeviceState>>,
    pub stop_flag: std::sync::Arc<std::sync::atomic::AtomicBool>,
    pub join_handle: Option<tokio::task::JoinHandle<()>>,
    pub last_saved_fif: Option<PathBuf>,
    pub record_started_at: Option<std::time::Instant>,
    pub last_record_toggle: Option<std::time::Instant>,
    pub epoch_seq: u32,
    /// Path to the session file for write-ahead logging.
    pub session_path: Option<PathBuf>,
}

// ── Semantic functions (pure, testable) ──────────────────────────────────────

/// Next available headband_id not used by any connected device.
pub fn next_available_headband_id(connected: &[ConnectedDevice]) -> Option<HeadbandId> {
    let used: HashSet<u32> = connected.iter().map(|d| d.headband_id.as_u32()).collect();
    (0..HeadbandId::MAX)
        .find(|id| !used.contains(id))
        .and_then(HeadbandId::new)
}

/// Clamp tab index after a device removal.
pub fn clamp_tab_index(current: usize, device_count: usize) -> usize {
    if device_count == 0 {
        0
    } else {
        current.min(device_count - 1)
    }
}

/// Check if all channel buffers have at least `chunk_size` samples ready.
pub fn chunk_ready(buffers: &[Vec<f32>], chunk_size: usize) -> bool {
    buffers.iter().all(|b| b.len() >= chunk_size)
}

/// Drain exactly `chunk_size` samples from the front of each buffer.
pub fn drain_chunk(buffers: &mut [Vec<f32>], chunk_size: usize) -> Vec<Vec<f32>> {
    buffers
        .iter_mut()
        .map(|b| b.drain(..chunk_size).collect())
        .collect()
}

/// Append samples to rolling buffer, trim to max_points.
pub fn push_rolling(buffer: &mut Vec<f32>, samples: &[f32], max_points: usize) {
    buffer.extend_from_slice(samples);
    let excess = buffer.len().saturating_sub(max_points);
    if excess > 0 {
        buffer.drain(..excess);
    }
}

/// Convert raw f32 samples to timed [time_secs, value] points.
/// Samples are spaced at 1/SAMPLE_RATE from `base_secs`.
fn samples_to_timed(samples: &[f32], base_secs: f64) -> Vec<[f64; 2]> {
    samples
        .iter()
        .enumerate()
        .map(|(i, &v)| [base_secs + i as f64 / SAMPLE_RATE, v as f64])
        .collect()
}

/// Append timed points to rolling buffer, trim to max_points.
fn push_rolling_timed(buffer: &mut Vec<[f64; 2]>, points: &[[f64; 2]], max_points: usize) {
    buffer.extend_from_slice(points);
    let excess = buffer.len().saturating_sub(max_points);
    if excess > 0 {
        buffer.drain(..excess);
    }
}

/// Append samples to recording accumulation buffer (no trimming).
pub fn push_accumulating(buffer: &mut Vec<f32>, samples: &[f32]) {
    buffer.extend_from_slice(samples);
}

/// Build an EegmFrame from a chunk of channel data.
pub fn build_eegm_frame(
    headband_id: HeadbandId,
    epoch_seq: u32,
    channels: &[Vec<f32>],
    timestamp_us: u64,
) -> EegmFrame {
    let n_samples = channels.first().map_or(0, |c| c.len());
    EegmFrame::with_timestamp(headband_id.as_u32(), epoch_seq, channels, n_samples, timestamp_us)
}

/// Build display label for a discovered device.
pub fn device_display_label(name: &str, id: &str) -> String {
    let suffix = if id.len() >= 4 {
        &id[id.len() - 4..]
    } else {
        id
    };
    format!("{name} (…{suffix})")
}

/// Generate FIF output path with device name for disambiguation.
pub fn device_fif_path(base_dir: &Path, device_name: &str) -> PathBuf {
    let dir = base_dir.join("live_captures");
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0);
    let safe_name = device_name
        .replace(|c: char| !c.is_alphanumeric() && c != '-', "_");
    dir.join(format!("live_{safe_name}_{ts}.fif"))
}

/// Calculate recording duration from sample count at known sample rate.
#[allow(dead_code)]
pub fn recording_duration_secs(sample_count: usize, sample_rate: f64) -> f64 {
    sample_count as f64 / sample_rate
}

/// Extract per-channel sample vectors from an EegmFrame.
pub fn extract_channels_from_frame(frame: &EegmFrame) -> Vec<Vec<f32>> {
    (0..frame.n_channels as usize)
        .map(|ch| frame.channel_data(ch).to_vec())
        .collect()
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── HeadbandId ───────────────────────────────────────────────────────

    #[test]
    fn headband_id_valid_range() {
        assert!(HeadbandId::new(0).is_some());
        assert!(HeadbandId::new(3).is_some());
        assert!(HeadbandId::new(4).is_none());
        assert!(HeadbandId::new(100).is_none());
    }

    #[test]
    fn headband_id_roundtrip() {
        let id = HeadbandId::new(2).unwrap();
        assert_eq!(id.as_u32(), 2);
    }

    // ── chunk_ready / drain_chunk ────────────────────────────────────────

    #[test]
    fn chunk_ready_all_sufficient() {
        let buffers = vec![vec![0.0; 10], vec![0.0; 10]];
        assert!(chunk_ready(&buffers, 10));
        assert!(!chunk_ready(&buffers, 11));
    }

    #[test]
    fn chunk_ready_one_short() {
        let buffers = vec![vec![0.0; 10], vec![0.0; 5]];
        assert!(!chunk_ready(&buffers, 10));
    }

    #[test]
    fn chunk_ready_empty() {
        let buffers: Vec<Vec<f32>> = vec![vec![], vec![]];
        assert!(!chunk_ready(&buffers, 1));
    }

    #[test]
    fn drain_chunk_basic() {
        let mut buffers = vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0]];
        let chunk = drain_chunk(&mut buffers, 2);
        assert_eq!(chunk, vec![vec![1.0, 2.0], vec![4.0, 5.0]]);
        assert_eq!(buffers, vec![vec![3.0], vec![6.0]]);
    }

    // ── push_rolling ────────────────────────────────────────────────────

    #[test]
    fn push_rolling_trims_excess() {
        let mut buf = vec![1.0, 2.0, 3.0];
        push_rolling(&mut buf, &[4.0, 5.0], 4);
        assert_eq!(buf, vec![2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn push_rolling_no_trim_when_under_limit() {
        let mut buf = vec![1.0];
        push_rolling(&mut buf, &[2.0], 10);
        assert_eq!(buf, vec![1.0, 2.0]);
    }

    // ── push_accumulating ───────────────────────────────────────────────

    #[test]
    fn push_accumulating_grows_unbounded() {
        let mut buf = vec![1.0];
        push_accumulating(&mut buf, &[2.0, 3.0]);
        assert_eq!(buf, vec![1.0, 2.0, 3.0]);
    }

    // ── clamp_tab_index ─────────────────────────────────────────────────

    #[test]
    fn clamp_tab_index_empty() {
        assert_eq!(clamp_tab_index(5, 0), 0);
    }

    #[test]
    fn clamp_tab_index_within_range() {
        assert_eq!(clamp_tab_index(1, 3), 1);
    }

    #[test]
    fn clamp_tab_index_over_range() {
        assert_eq!(clamp_tab_index(5, 3), 2);
    }

    // ── device_display_label ────────────────────────────────────────────

    #[test]
    fn display_label_long_id() {
        let label = device_display_label("Muse-AB12", "550e8400-e29b-41d4-a716-446655440000");
        assert_eq!(label, "Muse-AB12 (…0000)");
    }

    #[test]
    fn display_label_short_id() {
        let label = device_display_label("Muse", "AB");
        assert_eq!(label, "Muse (…AB)");
    }

    // ── device_fif_path ─────────────────────────────────────────────────

    #[test]
    fn fif_path_contains_device_name() {
        let path = device_fif_path(Path::new("/tmp/zuna"), "Muse-AB12");
        let name = path.file_name().unwrap().to_str().unwrap();
        assert!(name.starts_with("live_Muse-AB12_"));
        assert!(name.ends_with(".fif"));
        assert_eq!(path.parent().unwrap(), Path::new("/tmp/zuna/live_captures"));
    }

    #[test]
    fn fif_path_sanitizes_special_chars() {
        let path = device_fif_path(Path::new("/tmp"), "Muse (test)");
        let name = path.file_name().unwrap().to_str().unwrap();
        assert!(name.contains("Muse__test_"));
    }

    // ── recording_duration_secs ─────────────────────────────────────────

    #[test]
    fn recording_duration_basic() {
        let secs = recording_duration_secs(256, 256.0);
        assert!((secs - 1.0).abs() < f64::EPSILON);
    }

    #[test]
    fn recording_duration_zero() {
        assert_eq!(recording_duration_secs(0, 256.0), 0.0);
    }

    // ── build_eegm_frame ────────────────────────────────────────────────

    #[test]
    fn build_eegm_frame_basic() {
        let channels = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let hid = HeadbandId::new(1).unwrap();
        let frame = build_eegm_frame(hid, 42, &channels, 1_000_000);
        assert_eq!(frame.headband_id, 1);
        assert_eq!(frame.epoch_seq, 42);
        assert_eq!(frame.n_channels, 2);
        assert_eq!(frame.n_samples, 2);
        assert_eq!(frame.timestamp_us, 1_000_000);
    }

    // ── extract_channels_from_frame ─────────────────────────────────────

    #[test]
    fn extract_channels_roundtrip() {
        let channels = vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0]];
        let frame = EegmFrame::new(0, 0, &channels, 3);
        let extracted = extract_channels_from_frame(&frame);
        assert_eq!(extracted, channels);
    }

    // ── DeviceState ─────────────────────────────────────────────────────

    #[test]
    fn device_state_push_raw_trims() {
        let mut ds = DeviceState::new("Test".into(), "id".into(), 10);
        let frame = vec![vec![1.0; 8]; 4];
        ds.push_raw_frame(frame.clone(), 1_000_000);
        ds.push_raw_frame(frame, 2_000_000);
        // 16 samples pushed, max_points=10, should be trimmed to 10
        assert_eq!(ds.raw_channels[0].len(), 10);
        assert_eq!(ds.raw_frame_count, 2);
    }

    #[test]
    fn device_state_recording_accumulates() {
        let mut ds = DeviceState::new("Test".into(), "id".into(), 10);
        ds.start_recording();
        let frame = vec![vec![1.0; 5]; 4];
        ds.push_raw_frame(frame.clone(), 1_000_000);
        ds.push_raw_frame(frame, 2_000_000);
        // Recording buffer should have all 10 samples (unbounded)
        assert_eq!(ds.record_sample_count, 10);
        // Raw channels should also have 10 (at max_points)
        assert_eq!(ds.raw_channels[0].len(), 10);
    }

    #[test]
    fn device_state_take_recording_clears() {
        let mut ds = DeviceState::new("Test".into(), "id".into(), 100);
        ds.start_recording();
        ds.push_raw_frame(vec![vec![1.0; 5]; 4], 1_000_000);
        let (buf, count) = ds.take_recording();
        assert_eq!(count, 5);
        assert_eq!(buf.len(), 4);
        assert!(!ds.recording_active);
        assert_eq!(ds.record_sample_count, 0);
        assert!(ds.record_buffer.is_empty());
    }

    #[test]
    fn device_state_cancel_recording() {
        let mut ds = DeviceState::new("Test".into(), "id".into(), 100);
        ds.start_recording();
        ds.push_raw_frame(vec![vec![1.0; 5]; 4], 1_000_000);
        ds.cancel_recording();
        assert!(!ds.recording_active);
        assert_eq!(ds.record_sample_count, 0);
    }

    #[test]
    fn device_state_push_reconstructed() {
        let mut ds = DeviceState::new("Test".into(), "id".into(), 10);
        // Need raw data first for stream_start_us
        ds.push_raw_frame(vec![vec![0.0; 5]; 4], 1_000_000);
        let frame = vec![vec![1.0; 5]; 4];
        ds.push_reconstructed_frame(frame, 1_000_000);
        assert_eq!(ds.recon_channels.len(), 4);
        assert_eq!(ds.recon_channels[0].len(), 5);
        assert_eq!(ds.recon_frame_count, 1);
    }

    // ── Timestamp alignment tests ───────────────────────────────────────

    #[test]
    fn raw_frame_stores_timed_points() {
        let mut ds = DeviceState::new("T".into(), "id".into(), 1000);
        // First frame at t=1_000_000 µs (1 second after epoch)
        ds.push_raw_frame(vec![vec![10.0, 20.0]; 4], 1_000_000);
        // stream_start_us should be set to first frame
        let pts = &ds.raw_channels[0];
        assert_eq!(pts.len(), 2);
        // First sample at relative time 0.0s
        assert!((pts[0][0] - 0.0).abs() < 1e-9);
        assert!((pts[0][1] - 10.0).abs() < 1e-9);
        // Second sample at 1/256 s later
        assert!((pts[1][0] - 1.0 / 256.0).abs() < 1e-6);
        assert!((pts[1][1] - 20.0).abs() < 1e-9);
    }

    #[test]
    fn second_chunk_time_continues() {
        let mut ds = DeviceState::new("T".into(), "id".into(), 1000);
        // 256 samples at t=0 → occupies 0.0 to ~1.0 seconds
        ds.push_raw_frame(vec![vec![1.0; 256]; 4], 1_000_000);
        // Next 256 samples 1 second later
        ds.push_raw_frame(vec![vec![2.0; 256]; 4], 2_000_000);
        let pts = &ds.raw_channels[0];
        assert_eq!(pts.len(), 512);
        // First chunk starts at 0.0
        assert!((pts[0][0] - 0.0).abs() < 1e-9);
        // Second chunk starts at 1.0s (1_000_000 µs relative)
        assert!((pts[256][0] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn recon_aligns_with_raw_by_timestamp() {
        let mut ds = DeviceState::new("T".into(), "id".into(), 10000);
        // Raw at t=1_000_000 µs
        ds.push_raw_frame(vec![vec![1.0; 256]; 4], 1_000_000);
        // Recon arrives later but carries the SAME timestamp
        ds.push_reconstructed_frame(vec![vec![9.0; 256]; 4], 1_000_000);

        let raw_t0 = ds.raw_channels[0][0][0];
        let recon_t0 = ds.recon_channels[0][0][0];
        // Both should start at the same x-coordinate
        assert!((raw_t0 - recon_t0).abs() < 1e-9,
            "raw_t0={raw_t0}, recon_t0={recon_t0}");
    }

    #[test]
    fn recon_ts_zero_falls_back_to_end_of_raw() {
        let mut ds = DeviceState::new("T".into(), "id".into(), 10000);
        // Push 256 samples of raw data
        ds.push_raw_frame(vec![vec![1.0; 256]; 4], 1_000_000);
        let raw_last_t = ds.raw_channels[0].last().unwrap()[0];

        // Recon with ts=0 should fall back to end of raw
        ds.push_reconstructed_frame(vec![vec![9.0; 10]; 4], 0);
        let recon_t0 = ds.recon_channels[0][0][0];
        assert!((recon_t0 - raw_last_t).abs() < 1e-9,
            "recon should start at end of raw: recon_t0={recon_t0}, raw_last_t={raw_last_t}");
    }

    // ── next_available_headband_id (needs ConnectedDevice stubs) ────────

    fn stub_connected_device(hid: u32) -> ConnectedDevice {
        let hid = HeadbandId::new(hid).unwrap();
        ConnectedDevice {
            device_name: String::new(),
            device_id: String::new(),
            headband_id: hid,
            state: std::sync::Arc::new(std::sync::Mutex::new(
                DeviceState::new(String::new(), String::new(), 100),
            )),
            stop_flag: std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)),
            join_handle: None,
            last_saved_fif: None,
            record_started_at: None,
            last_record_toggle: None,
            epoch_seq: 0,
            session_path: None,
        }
    }

    #[test]
    fn next_headband_id_empty() {
        let id = next_available_headband_id(&[]);
        assert_eq!(id.unwrap().as_u32(), 0);
    }

    #[test]
    fn next_headband_id_skips_used() {
        let devices = vec![stub_connected_device(0), stub_connected_device(2)];
        let id = next_available_headband_id(&devices);
        assert_eq!(id.unwrap().as_u32(), 1);
    }

    #[test]
    fn next_headband_id_all_used() {
        let devices: Vec<_> = (0..4).map(stub_connected_device).collect();
        assert!(next_available_headband_id(&devices).is_none());
    }

    // ── SessionWriter ───────────────────────────────────────────────────

    fn make_test_frame(headband_id: u32, epoch_seq: u32) -> EegmFrame {
        let channels = vec![vec![1.0f32, 2.0], vec![3.0, 4.0]];
        EegmFrame::new(headband_id, epoch_seq, &channels, 2)
    }

    #[test]
    fn session_writer_create_and_append() {
        let dir = std::env::temp_dir().join("eeg_test_session_create");
        let _ = std::fs::remove_dir_all(&dir);
        let path = dir.join("sessions").join("test_device_123.eegm");

        let mut sw = SessionWriter::create(&path).unwrap();
        assert_eq!(sw.frames_written(), 0);

        sw.append_frame(&make_test_frame(0, 0)).unwrap();
        sw.append_frame(&make_test_frame(0, 1)).unwrap();

        assert_eq!(sw.frames_written(), 2);
        assert!(path.is_file());

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn read_session_frames_all() {
        let dir = std::env::temp_dir().join("eeg_test_read_all");
        let _ = std::fs::remove_dir_all(&dir);
        let path = dir.join("test.eegm");

        let mut sw = SessionWriter::create(&path).unwrap();
        sw.append_frame(&make_test_frame(1, 10)).unwrap();
        sw.append_frame(&make_test_frame(1, 11)).unwrap();
        sw.append_frame(&make_test_frame(1, 12)).unwrap();
        drop(sw);

        let frames = read_session_frames(&path, 0).unwrap();
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0].epoch_seq, 10);
        assert_eq!(frames[1].epoch_seq, 11);
        assert_eq!(frames[2].epoch_seq, 12);
        assert_eq!(frames[0].headband_id, 1);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn read_session_frames_from_offset() {
        let dir = std::env::temp_dir().join("eeg_test_read_offset");
        let _ = std::fs::remove_dir_all(&dir);
        let path = dir.join("test.eegm");

        let mut sw = SessionWriter::create(&path).unwrap();
        sw.append_frame(&make_test_frame(0, 0)).unwrap();
        sw.append_frame(&make_test_frame(0, 1)).unwrap();
        sw.append_frame(&make_test_frame(0, 2)).unwrap();
        sw.append_frame(&make_test_frame(0, 3)).unwrap();
        drop(sw);

        // Skip first 2, read from frame index 2
        let frames = read_session_frames(&path, 2).unwrap();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].epoch_seq, 2);
        assert_eq!(frames[1].epoch_seq, 3);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn read_session_frames_empty_file() {
        let dir = std::env::temp_dir().join("eeg_test_read_empty");
        let _ = std::fs::remove_dir_all(&dir);
        let path = dir.join("test.eegm");

        let _ = std::fs::create_dir_all(&dir);
        std::fs::File::create(&path).unwrap();

        let frames = read_session_frames(&path, 0).unwrap();
        assert!(frames.is_empty());

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn read_session_frames_roundtrip_data() {
        let dir = std::env::temp_dir().join("eeg_test_roundtrip_data");
        let _ = std::fs::remove_dir_all(&dir);
        let path = dir.join("test.eegm");

        let channels = vec![vec![10.0f32, 20.0, 30.0], vec![40.0, 50.0, 60.0]];
        let frame = EegmFrame::new(2, 99, &channels, 3);

        let mut sw = SessionWriter::create(&path).unwrap();
        sw.append_frame(&frame).unwrap();
        drop(sw);

        let read = read_session_frames(&path, 0).unwrap();
        assert_eq!(read.len(), 1);
        let f = &read[0];
        assert_eq!(f.headband_id, 2);
        assert_eq!(f.epoch_seq, 99);
        assert_eq!(f.n_channels, 2);
        assert_eq!(f.n_samples, 3);
        let extracted = extract_channels_from_frame(f);
        assert_eq!(extracted, channels);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn session_file_path_format() {
        let path = session_file_path(Path::new("/tmp/zuna"), "Muse-AB12");
        assert_eq!(path.parent().unwrap(), Path::new("/tmp/zuna/sessions"));
        let name = path.file_name().unwrap().to_str().unwrap();
        assert!(name.starts_with("Muse-AB12_"));
        assert!(name.ends_with(".eegm"));
    }

    #[test]
    fn session_file_path_sanitizes() {
        let path = session_file_path(Path::new("/tmp"), "Muse (test)");
        let name = path.file_name().unwrap().to_str().unwrap();
        assert!(name.contains("Muse__test_"));
    }

    // ── SharedOutboundTx ─────────────────────────────────────────────────

    #[tokio::test]
    async fn shared_outbound_tx_send_before_reset_returns_false() {
        let shared = SharedOutboundTx::new();
        let frame = make_test_frame(0, 0);
        assert!(!shared.send(frame), "send should fail when no channel set");
    }

    #[tokio::test]
    async fn shared_outbound_tx_send_after_reset_delivers() {
        let shared = SharedOutboundTx::new();
        let mut rx = shared.reset_channel();
        let frame = make_test_frame(0, 42);
        assert!(shared.send(frame), "send should succeed after reset");
        let received = rx.recv().await.unwrap();
        assert_eq!(received.epoch_seq, 42);
    }

    #[tokio::test]
    async fn shared_outbound_tx_clone_shares_channel() {
        let shared = SharedOutboundTx::new();
        let clone = shared.clone();
        let mut rx = shared.reset_channel();

        // Send through the clone — should arrive on the same rx
        let frame = make_test_frame(1, 7);
        assert!(clone.send(frame), "clone should share the channel");
        let received = rx.recv().await.unwrap();
        assert_eq!(received.headband_id, 1);
        assert_eq!(received.epoch_seq, 7);
    }

    #[tokio::test]
    async fn shared_outbound_tx_reset_replaces_channel() {
        let shared = SharedOutboundTx::new();
        let _rx1 = shared.reset_channel();

        // Reset again — old rx is dropped, new one works
        let mut rx2 = shared.reset_channel();
        let frame = make_test_frame(0, 99);
        assert!(shared.send(frame));
        let received = rx2.recv().await.unwrap();
        assert_eq!(received.epoch_seq, 99);
    }
}
