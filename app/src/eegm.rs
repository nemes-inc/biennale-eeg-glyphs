//! **EEGM** — EEG Multi-headband binary frame protocol.
//!
//! Extends the existing EEGF format with a `headband_id` and `epoch_seq`
//! field for multi-device streaming between `eeg-hub` and `inference-server`.
//!
//! ## Wire format (little-endian)
//!
//! | Offset | Type  | Value                                              |
//! |--------|-------|----------------------------------------------------|
//! | 0      | `u32` | Magic: `0x4545474D` (ASCII `"EEGM"`)               |
//! | 4      | `u32` | `headband_id` (0–3)                                |
//! | 8      | `u32` | `epoch_seq` — monotonic sequence per headband       |
//! | 12     | `u32` | `n_channels`                                       |
//! | 16     | `u32` | `n_samples` per channel                            |
//! | 20     | `f32[n_channels × n_samples]` | Channel-major payload |
//!
//! Total frame size: `20 + 4 × n_channels × n_samples` bytes.
//!
//! Direction determines semantics:
//! - **Hub → Server**: raw EEG from headband N
//! - **Server → Hub**: reconstructed EEG for headband N (may have more channels)
//!
//! ## Control messages (EEGC)
//!
//! A separate magic `0x45454743` (`"EEGC"`) identifies fixed-size control
//! messages used for connection handshake:
//!
//! | Offset | Type  | Value                                     |
//! |--------|-------|-------------------------------------------|
//! | 0      | `u32` | Magic: `0x45454743` (ASCII `"EEGC"`)      |
//! | 4      | `u32` | `msg_type` (1 = CONNECT\_REQ, 2 = CONNECT\_ACK) |
//! | 8      | `u32` | `protocol_version` (currently 1)          |
//! | 12     | `u32` | `n_headbands` (1–4)                       |
//! | 16     | `u32` | `sample_rate` (e.g. 256)                  |
//! | 20     | `u32` | `status` (ACK only: 0 = OK, non-zero = error) |
//!
//! Handshake sequence:
//! 1. Hub connects via TCP
//! 2. Hub sends `ConnectReq`
//! 3. Server validates and replies with `ConnectAck`
//! 4. If `status == 0`, hub begins streaming `EegmFrame` data

use std::io::{self, Read, Write};

/// EEGM magic: ASCII "EEGM" = 0x4545_474D (little-endian).
pub const MAGIC_EEGM: u32 = 0x4545_474D;

/// EEGC control magic: ASCII "EEGC" = 0x4545_4743 (little-endian).
pub const MAGIC_EEGC: u32 = 0x4545_4743;

/// Header size in bytes for data frames.
pub const HEADER_SIZE: usize = 20;

/// Fixed size of a control message (EEGC): 6 × u32 = 24 bytes.
pub const CTRL_SIZE: usize = 24;

/// Maximum supported headband count.
pub const MAX_HEADBANDS: u32 = 4;

/// Current protocol version.
pub const PROTOCOL_VERSION: u32 = 1;

// ── Control message types ────────────────────────────────────────────────────

const MSG_CONNECT_REQ: u32 = 1;
const MSG_CONNECT_ACK: u32 = 2;

/// Connection request sent by the hub to the inference server.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConnectReq {
    pub protocol_version: u32,
    pub n_headbands: u32,
    pub sample_rate: u32,
}

impl ConnectReq {
    pub fn new(n_headbands: u32, sample_rate: u32) -> Self {
        Self {
            protocol_version: PROTOCOL_VERSION,
            n_headbands,
            sample_rate,
        }
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(CTRL_SIZE);
        buf.extend_from_slice(&MAGIC_EEGC.to_le_bytes());
        buf.extend_from_slice(&MSG_CONNECT_REQ.to_le_bytes());
        buf.extend_from_slice(&self.protocol_version.to_le_bytes());
        buf.extend_from_slice(&self.n_headbands.to_le_bytes());
        buf.extend_from_slice(&self.sample_rate.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes()); // reserved
        buf
    }

    pub fn write_to<W: Write>(&self, w: &mut W) -> io::Result<()> {
        w.write_all(&self.encode())
    }
}

/// Connection acknowledgement sent by the server back to the hub.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConnectAck {
    pub protocol_version: u32,
    pub n_headbands: u32,
    pub sample_rate: u32,
    /// 0 = OK, non-zero = error code.
    pub status: u32,
}

impl ConnectAck {
    pub fn ok(n_headbands: u32, sample_rate: u32) -> Self {
        Self {
            protocol_version: PROTOCOL_VERSION,
            n_headbands,
            sample_rate,
            status: 0,
        }
    }

    pub fn error(code: u32) -> Self {
        Self {
            protocol_version: PROTOCOL_VERSION,
            n_headbands: 0,
            sample_rate: 0,
            status: code,
        }
    }

    pub fn is_ok(&self) -> bool {
        self.status == 0
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(CTRL_SIZE);
        buf.extend_from_slice(&MAGIC_EEGC.to_le_bytes());
        buf.extend_from_slice(&MSG_CONNECT_ACK.to_le_bytes());
        buf.extend_from_slice(&self.protocol_version.to_le_bytes());
        buf.extend_from_slice(&self.n_headbands.to_le_bytes());
        buf.extend_from_slice(&self.sample_rate.to_le_bytes());
        buf.extend_from_slice(&self.status.to_le_bytes());
        buf
    }

    pub fn write_to<W: Write>(&self, w: &mut W) -> io::Result<()> {
        w.write_all(&self.encode())
    }
}

// ── Top-level message enum ───────────────────────────────────────────────────

/// A message read from the wire — either a control handshake or a data frame.
#[derive(Debug, Clone)]
pub enum EegmMessage {
    ConnectReq(ConnectReq),
    ConnectAck(ConnectAck),
    Data(EegmFrame),
}

impl EegmMessage {
    /// Read the next message from a reader, dispatching on the magic bytes.
    ///
    /// Returns `Ok(None)` on clean EOF.
    pub fn read_from<R: Read>(r: &mut R) -> io::Result<Option<Self>> {
        let mut magic_buf = [0u8; 4];
        match r.read_exact(&mut magic_buf) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(e) => return Err(e),
        }
        let magic = u32::from_le_bytes(magic_buf);

        match magic {
            MAGIC_EEGC => {
                // Control message: read remaining 20 bytes (CTRL_SIZE - 4)
                let mut rest = [0u8; CTRL_SIZE - 4];
                r.read_exact(&mut rest)?;
                let msg_type = u32::from_le_bytes([rest[0], rest[1], rest[2], rest[3]]);
                let version = u32::from_le_bytes([rest[4], rest[5], rest[6], rest[7]]);
                let n_headbands = u32::from_le_bytes([rest[8], rest[9], rest[10], rest[11]]);
                let sample_rate = u32::from_le_bytes([rest[12], rest[13], rest[14], rest[15]]);
                let status = u32::from_le_bytes([rest[16], rest[17], rest[18], rest[19]]);

                match msg_type {
                    MSG_CONNECT_REQ => Ok(Some(EegmMessage::ConnectReq(ConnectReq {
                        protocol_version: version,
                        n_headbands,
                        sample_rate,
                    }))),
                    MSG_CONNECT_ACK => Ok(Some(EegmMessage::ConnectAck(ConnectAck {
                        protocol_version: version,
                        n_headbands,
                        sample_rate,
                        status,
                    }))),
                    _ => Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("unknown EEGC msg_type: {msg_type}"),
                    )),
                }
            }
            MAGIC_EEGM => {
                // Data frame: read remaining header bytes then payload
                let mut hdr_rest = [0u8; HEADER_SIZE - 4];
                r.read_exact(&mut hdr_rest)?;
                let headband_id = u32::from_le_bytes([hdr_rest[0], hdr_rest[1], hdr_rest[2], hdr_rest[3]]);
                let epoch_seq = u32::from_le_bytes([hdr_rest[4], hdr_rest[5], hdr_rest[6], hdr_rest[7]]);
                let n_channels = u32::from_le_bytes([hdr_rest[8], hdr_rest[9], hdr_rest[10], hdr_rest[11]]);
                let n_samples = u32::from_le_bytes([hdr_rest[12], hdr_rest[13], hdr_rest[14], hdr_rest[15]]);

                if n_channels > 64 || n_samples > 65536 {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidData,
                        format!("implausible EEGM dimensions: {n_channels} ch × {n_samples} samples"),
                    ));
                }

                let payload_len = (n_channels as usize) * (n_samples as usize);
                let mut raw = vec![0u8; payload_len * 4];
                r.read_exact(&mut raw)?;
                let data: Vec<f32> = raw
                    .chunks_exact(4)
                    .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                    .collect();

                Ok(Some(EegmMessage::Data(EegmFrame {
                    headband_id,
                    epoch_seq,
                    n_channels,
                    n_samples,
                    data,
                })))
            }
            _ => Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("unknown magic: 0x{magic:08X} (expected EEGM or EEGC)"),
            )),
        }
    }
}

/// Decoded EEGM frame.
#[derive(Debug, Clone)]
pub struct EegmFrame {
    /// Which headband this frame belongs to (0–3).
    pub headband_id: u32,
    /// Monotonically increasing epoch sequence number per headband.
    pub epoch_seq: u32,
    /// Number of channels in this frame.
    pub n_channels: u32,
    /// Number of samples per channel.
    pub n_samples: u32,
    /// Channel-major payload: all samples for ch0, then ch1, …
    /// Length = n_channels × n_samples.
    pub data: Vec<f32>,
}

impl EegmFrame {
    /// Create a new frame from channel vectors.
    ///
    /// `channels` must all have the same length ≥ `n_samples`.
    pub fn new(
        headband_id: u32,
        epoch_seq: u32,
        channels: &[Vec<f32>],
        n_samples: usize,
    ) -> Self {
        let n_channels = channels.len() as u32;
        let mut data = Vec::with_capacity(channels.len() * n_samples);
        for ch in channels {
            data.extend_from_slice(&ch[..n_samples]);
        }
        Self {
            headband_id,
            epoch_seq,
            n_channels,
            n_samples: n_samples as u32,
            data,
        }
    }

    /// Total frame size in bytes (header + payload).
    pub fn wire_size(&self) -> usize {
        HEADER_SIZE + (self.n_channels as usize) * (self.n_samples as usize) * 4
    }

    /// Encode this frame into a byte buffer.
    pub fn encode(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(self.wire_size());
        buf.extend_from_slice(&MAGIC_EEGM.to_le_bytes());
        buf.extend_from_slice(&self.headband_id.to_le_bytes());
        buf.extend_from_slice(&self.epoch_seq.to_le_bytes());
        buf.extend_from_slice(&self.n_channels.to_le_bytes());
        buf.extend_from_slice(&self.n_samples.to_le_bytes());
        for &sample in &self.data {
            buf.extend_from_slice(&sample.to_le_bytes());
        }
        buf
    }

    /// Write this frame to a writer.
    pub fn write_to<W: Write>(&self, w: &mut W) -> io::Result<()> {
        w.write_all(&self.encode())
    }

    /// Read one EEGM frame from a reader.
    ///
    /// Returns `Ok(None)` on clean EOF (0 bytes read for magic).
    /// Returns `Err` on malformed data or partial read.
    pub fn read_from<R: Read>(r: &mut R) -> io::Result<Option<Self>> {
        // Read header
        let mut hdr = [0u8; HEADER_SIZE];
        match r.read_exact(&mut hdr[..4]) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(e) => return Err(e),
        }
        r.read_exact(&mut hdr[4..])?;

        let magic = u32::from_le_bytes([hdr[0], hdr[1], hdr[2], hdr[3]]);
        if magic != MAGIC_EEGM {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("bad EEGM magic: 0x{magic:08X} (expected 0x{MAGIC_EEGM:08X})"),
            ));
        }

        let headband_id = u32::from_le_bytes([hdr[4], hdr[5], hdr[6], hdr[7]]);
        let epoch_seq = u32::from_le_bytes([hdr[8], hdr[9], hdr[10], hdr[11]]);
        let n_channels = u32::from_le_bytes([hdr[12], hdr[13], hdr[14], hdr[15]]);
        let n_samples = u32::from_le_bytes([hdr[16], hdr[17], hdr[18], hdr[19]]);

        // Sanity limits
        if n_channels > 64 || n_samples > 65536 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("implausible EEGM dimensions: {n_channels} ch × {n_samples} samples"),
            ));
        }

        let payload_len = (n_channels as usize) * (n_samples as usize);
        let mut raw = vec![0u8; payload_len * 4];
        r.read_exact(&mut raw)?;

        let data: Vec<f32> = raw
            .chunks_exact(4)
            .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .collect();

        Ok(Some(Self {
            headband_id,
            epoch_seq,
            n_channels,
            n_samples,
            data,
        }))
    }

    /// Extract samples for a specific channel (0-indexed).
    /// Returns a slice into `self.data`.
    pub fn channel_data(&self, ch: usize) -> &[f32] {
        let start = ch * self.n_samples as usize;
        let end = start + self.n_samples as usize;
        &self.data[start..end]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    #[test]
    fn connect_roundtrip() {
        let req = ConnectReq::new(2, 256);
        let encoded = req.encode();
        assert_eq!(encoded.len(), CTRL_SIZE);

        let mut cursor = Cursor::new(&encoded);
        match EegmMessage::read_from(&mut cursor).unwrap().unwrap() {
            EegmMessage::ConnectReq(decoded) => {
                assert_eq!(decoded.protocol_version, PROTOCOL_VERSION);
                assert_eq!(decoded.n_headbands, 2);
                assert_eq!(decoded.sample_rate, 256);
            }
            other => panic!("expected ConnectReq, got {other:?}"),
        }
    }

    #[test]
    fn connect_ack_roundtrip() {
        let ack = ConnectAck::ok(4, 256);
        assert!(ack.is_ok());
        let encoded = ack.encode();

        let mut cursor = Cursor::new(&encoded);
        match EegmMessage::read_from(&mut cursor).unwrap().unwrap() {
            EegmMessage::ConnectAck(decoded) => {
                assert_eq!(decoded.status, 0);
                assert_eq!(decoded.n_headbands, 4);
                assert_eq!(decoded.sample_rate, 256);
                assert!(decoded.is_ok());
            }
            other => panic!("expected ConnectAck, got {other:?}"),
        }

        // Error ack
        let err_ack = ConnectAck::error(1);
        assert!(!err_ack.is_ok());
        let encoded = err_ack.encode();
        let mut cursor = Cursor::new(&encoded);
        match EegmMessage::read_from(&mut cursor).unwrap().unwrap() {
            EegmMessage::ConnectAck(decoded) => assert_eq!(decoded.status, 1),
            other => panic!("expected ConnectAck, got {other:?}"),
        }
    }

    #[test]
    fn mixed_message_stream() {
        let mut buf = Vec::new();
        ConnectReq::new(1, 256).write_to(&mut buf).unwrap();
        let ch = vec![vec![1.0f32; 4]; 4];
        EegmFrame::new(0, 0, &ch, 4).write_to(&mut buf).unwrap();
        ConnectAck::ok(1, 256).write_to(&mut buf).unwrap();

        let mut cursor = Cursor::new(&buf);
        assert!(matches!(
            EegmMessage::read_from(&mut cursor).unwrap().unwrap(),
            EegmMessage::ConnectReq(_)
        ));
        assert!(matches!(
            EegmMessage::read_from(&mut cursor).unwrap().unwrap(),
            EegmMessage::Data(_)
        ));
        assert!(matches!(
            EegmMessage::read_from(&mut cursor).unwrap().unwrap(),
            EegmMessage::ConnectAck(_)
        ));
        assert!(EegmMessage::read_from(&mut cursor).unwrap().is_none());
    }

    #[test]
    fn roundtrip() {
        let channels = vec![
            vec![1.0f32, 2.0, 3.0, 4.0],
            vec![5.0, 6.0, 7.0, 8.0],
            vec![9.0, 10.0, 11.0, 12.0],
            vec![13.0, 14.0, 15.0, 16.0],
        ];
        let frame = EegmFrame::new(2, 42, &channels, 4);
        assert_eq!(frame.headband_id, 2);
        assert_eq!(frame.epoch_seq, 42);
        assert_eq!(frame.n_channels, 4);
        assert_eq!(frame.n_samples, 4);
        assert_eq!(frame.data.len(), 16);

        let encoded = frame.encode();
        assert_eq!(encoded.len(), HEADER_SIZE + 16 * 4);

        let mut cursor = Cursor::new(&encoded);
        let decoded = EegmFrame::read_from(&mut cursor).unwrap().unwrap();
        assert_eq!(decoded.headband_id, 2);
        assert_eq!(decoded.epoch_seq, 42);
        assert_eq!(decoded.n_channels, 4);
        assert_eq!(decoded.n_samples, 4);
        assert_eq!(decoded.data, frame.data);
    }

    #[test]
    fn channel_data_access() {
        let channels = vec![
            vec![1.0f32, 2.0, 3.0],
            vec![4.0, 5.0, 6.0],
        ];
        let frame = EegmFrame::new(0, 0, &channels, 3);
        assert_eq!(frame.channel_data(0), &[1.0, 2.0, 3.0]);
        assert_eq!(frame.channel_data(1), &[4.0, 5.0, 6.0]);
    }

    #[test]
    fn eof_returns_none() {
        let mut cursor = Cursor::new(Vec::<u8>::new());
        assert!(EegmFrame::read_from(&mut cursor).unwrap().is_none());
    }

    #[test]
    fn bad_magic_errors() {
        let mut bad = vec![0u8; HEADER_SIZE + 16];
        bad[0..4].copy_from_slice(&0xDEADBEEFu32.to_le_bytes());
        let mut cursor = Cursor::new(&bad);
        assert!(EegmFrame::read_from(&mut cursor).is_err());
    }

    #[test]
    fn multi_frame_stream() {
        let mut buf = Vec::new();
        for i in 0..5u32 {
            let ch = vec![vec![i as f32; 8]; 4];
            let frame = EegmFrame::new(i % 4, i, &ch, 8);
            frame.write_to(&mut buf).unwrap();
        }

        let mut cursor = Cursor::new(&buf);
        for i in 0..5u32 {
            let frame = EegmFrame::read_from(&mut cursor).unwrap().unwrap();
            assert_eq!(frame.headband_id, i % 4);
            assert_eq!(frame.epoch_seq, i);
            assert_eq!(frame.n_samples, 8);
        }
        assert!(EegmFrame::read_from(&mut cursor).unwrap().is_none());
    }
}
