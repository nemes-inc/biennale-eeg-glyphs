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

use std::io::{self, Read, Write};

/// EEGM magic: ASCII "EEGM" = 0x4545_474D (little-endian).
pub const MAGIC_EEGM: u32 = 0x4545_474D;

/// Header size in bytes: magic(4) + headband_id(4) + epoch_seq(4) + n_channels(4) + n_samples(4).
pub const HEADER_SIZE: usize = 20;

/// Maximum supported headband count.
pub const MAX_HEADBANDS: u32 = 4;

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
