# Muse Headset — Available Parameters & Data Streams

Comprehensive reference of all data available from Muse headsets (Classic + Athena),
covering what is currently implemented in `muse-rs`, what needs client-side computation,
and what could be added.

---

## 1. Raw Sensor Streams (from BLE)

These arrive as BLE notifications and are decoded into `MuseEvent` variants.

### 1.1 EEG — `MuseEvent::Eeg(EegReading)`

| Property | Classic | Athena |
|---|---|---|
| Channels | 4 (TP9, AF7, AF8, TP10) + optional AUX | 4 or 8 (+ FPz, AUX_R, AUX_L, AUX) |
| Sample rate | 256 Hz | 256 Hz |
| Samples/packet | 12 | 2 (8ch) or 4 (4ch) |
| Resolution | 12-bit BE | 14-bit LE |
| Scale | 0.48828125 µV/LSB | 0.0885 µV/LSB |
| Unit | µV (microvolts) | µV (microvolts) |

**Electrode positions (10-20 system):**

| Index | Name | Location | Available |
|---|---|---|---|
| 0 | TP9 | Left ear (temporal-parietal) | Classic + Athena |
| 1 | AF7 | Left forehead (anterior-frontal) | Classic + Athena |
| 2 | AF8 | Right forehead (anterior-frontal) | Classic + Athena |
| 3 | TP10 | Right ear (temporal-parietal) | Classic + Athena |
| 4 | FPz | Frontal midline | Athena only |
| 5 | AUX_R | Right auxiliary | Athena only |
| 6 | AUX_L | Left auxiliary | Athena only |
| 7 | AUX | Auxiliary input | Classic (opt-in) / Athena |

**Status in muse-rs:** ✅ Fully implemented. Decoded, timestamped, displayed in TUI.

---

### 1.2 PPG (Photoplethysmography) — `MuseEvent::Ppg(PpgReading)`

| Property | Classic | Athena |
|---|---|---|
| Channels | 3 (ambient, infrared, red) | 3–4 (ambient, infrared, red, [unused]) |
| Sample rate | 64 Hz | 64 Hz |
| Samples/packet | 6 | 3 |
| Resolution | 24-bit BE unsigned | 20-bit LE unsigned |
| Unit | Raw ADC counts (not scaled) | Raw ADC counts |

Available on Muse 2 and Muse S only.

**Status in muse-rs:** ✅ Fully implemented. Displayed in TUI PPG view.

---

### 1.3 Accelerometer — `MuseEvent::Accelerometer(ImuData)`

| Property | Value |
|---|---|
| Axes | X, Y, Z |
| Sample rate | ~52 Hz |
| Samples/packet | 3 |
| Scale | 0.0000610352 g/LSB (±2G full-scale) |
| Unit | g (1g = 9.81 m/s²) |

**Orientation when worn on level head:**
- **X** — head tilt forward/backward (pitch)
- **Y** — head tilt left/right (roll)
- **Z** — vertical motion up/down; resting ≈ −1g (gravity)

**Status in muse-rs:** ✅ Parsed. Shown as text in TUI footer. ⚠️ No chart visualization yet.

---

### 1.4 Gyroscope — `MuseEvent::Gyroscope(ImuData)`

| Property | Classic | Athena |
|---|---|---|
| Axes | X, Y, Z | X, Y, Z |
| Sample rate | ~52 Hz | ~52 Hz |
| Scale | +0.0074768 °/s/LSB | −0.0074768 °/s/LSB (negated) |
| Unit | °/s (degrees per second) | °/s |

**Orientation when worn:**
- **X** — roll (head tilt left/right)
- **Y** — pitch (head tilt up/down)
- **Z** — yaw (head rotation left/right)

**Status in muse-rs:** ✅ Parsed. Shown as text in TUI footer. ⚠️ No chart visualization yet.

---

### 1.5 Telemetry / Battery — `MuseEvent::Telemetry(TelemetryData)`

| Field | Classic | Athena | Unit |
|---|---|---|---|
| `sequence_id` | from packet | always 0 | counter |
| `battery_level` | u16 BE ÷ 512 | u16 LE ÷ 256 | % (0–100) |
| `fuel_gauge_voltage` | u16 BE × 2.2 | always 0.0 | mV |
| `temperature` | u16 BE raw ADC | always 0 | raw (not °C) |

Rate: ~1 Hz (Classic), ~0.2 Hz (Athena tag 0x88) or ~1 Hz (Athena tag 0x98).

**Status in muse-rs:** ✅ Parsed. Battery % shown in TUI header. ⚠️ No dedicated panel.

---

### 1.6 Control / Status — `MuseEvent::Control(ControlResponse)`

JSON objects sent by the headset in response to commands or spontaneously during streaming.
Split across multiple BLE notifications, reassembled by `ControlAccumulator`.

#### Device Info (response to `v1` command)

| JSON Key | Description | Example |
|---|---|---|
| `rc` | Return code (0 = success) | `0` |
| `fw` | Firmware version | `"3.4.5"` |
| `hw` | Hardware revision | `"10.0"` |
| `bn` | Build number | `1234` |
| `bl` | Bootloader version | `"1.2.3"` |
| `pv` | Protocol version | `2` |
| `sp` | Serial port / product | `"RevE"` |
| `tp` | Headset type | `"Muse-S"` |
| `ap` | Application version | `"1.0.0"` |
| `hn` | Hostname (device name) | `"Muse-AB12"` |
| `sn` | Serial number | `"XXXX-XXXX"` |
| `ma` | MAC address | `"AA:BB:CC:DD:EE:FF"` |

#### Status (response to `s` command)

| JSON Key | Description | Notes |
|---|---|---|
| `rc` | Return code | `0` |
| `bp` | Battery percentage | Matches telemetry reading |
| `ts` | Timestamp / uptime | Device clock |
| Various | Other status fields | Firmware-specific |

#### Streaming status (sent spontaneously)

During streaming, the headset periodically sends control JSON with status updates.
The exact fields depend on firmware version.

**Status in muse-rs:** ✅ Parsed as raw JSON. Printed in CLI. ⚠️ Not displayed in TUI.

---

## 2. Derived / Computed Data (client-side)

These are NOT sent by the headset — they must be computed from raw sensor data.

### 2.1 Contact Quality / Horseshoe Indicator

The Muse SDK's "horseshoe" indicator shows electrode contact quality per channel.
It is **computed client-side** from the raw EEG signal, not sent by the device.

**Algorithm (from Muse SDK / Mind Monitor):**

Each electrode gets a quality score: 1 = good, 2 = ok, 4 = bad.

Heuristics for computing contact quality from raw EEG:

| Method | Description | Threshold |
|---|---|---|
| **RMS amplitude** | Root mean square of recent samples (~1s window). High RMS = noise = poor contact. | Good: < 20 µV, OK: 20–100 µV, Bad: > 100 µV |
| **Signal railing** | Percentage of samples at ADC min/max. Railing = no contact. | Good: < 5%, Bad: > 20% |
| **Variance** | Standard deviation of recent window. Very low = no signal, very high = noise. | Empirical thresholds |
| **Frequency content** | Ratio of high-frequency (>40 Hz) power to total power. High ratio = EMG/noise. | Empirical |

A practical approach: use a sliding 1-second window RMS per channel and classify into
Good / OK / Bad tiers. Combine with railing detection for "no contact" state.

**Status in muse-rs:** ❌ Not implemented. Needs new computation module.

---

### 2.2 Eye Blink Detection

Eye blinks produce a characteristic large-amplitude artifact (100–300 µV) on the
**frontal channels** (AF7, AF8), lasting approximately 200–400 ms.

**Detection algorithm:**

1. Monitor AF7 and AF8 channels
2. Apply a short sliding window (e.g., 100 ms)
3. Detect when peak-to-peak amplitude exceeds a threshold (~100 µV)
4. Require both AF7 and AF8 to spike simultaneously (distinguishes from lateral eye movement)
5. Enforce a minimum refractory period (~300 ms) to avoid double-counting
6. Optionally check that TP9/TP10 do NOT show the same artifact (confirms frontal origin)

**Characteristics:**
- Duration: 200–400 ms
- Amplitude: 100–300 µV (much larger than typical EEG ~10–50 µV)
- Location: bilateral frontal (AF7 + AF8)
- Shape: sharp positive deflection followed by negative rebound

**Status in muse-rs:** ❌ Not implemented. Needs new computation module.

---

### 2.3 Jaw Clench Detection

Jaw clenching produces sustained high-frequency EMG (electromyographic) artifact
primarily on the **temporal channels** (TP9, TP10).

**Detection algorithm:**

1. Monitor TP9 and TP10 channels
2. Compute high-frequency power (30–50 Hz band) using a sliding FFT or bandpass RMS
3. Detect when high-frequency power exceeds a threshold for a sustained period (~200 ms)
4. Require bilateral activation (both TP9 and TP10)
5. Distinguish from blink artifacts by checking frequency content (jaw = broadband HF, blink = low freq)

**Characteristics:**
- Duration: 200 ms to several seconds (as long as the clench is held)
- Frequency: broadband high-frequency (30–100+ Hz)
- Amplitude: 50–200 µV RMS in HF band
- Location: bilateral temporal (TP9 + TP10)

**Status in muse-rs:** ❌ Not implemented. Needs new computation module.

---

### 2.4 Frequency Band Powers (future)

Standard EEG frequency bands, computed via FFT or bandpass filtering:

| Band | Frequency | Associated state |
|---|---|---|
| Delta (δ) | 1–4 Hz | Deep sleep |
| Theta (θ) | 4–8 Hz | Drowsiness, meditation |
| Alpha (α) | 8–13 Hz | Relaxed, eyes closed |
| Beta (β) | 13–30 Hz | Active thinking, focus |
| Gamma (γ) | 30–44 Hz | Higher cognitive function |

Mind Monitor and the Muse SDK both compute absolute and relative band powers
from the PSD (Power Spectral Density) of the EEG signal per channel.

**Status in muse-rs:** ❌ Not implemented. Future enhancement.

---

## 3. Implementation Status Summary

| Data | Source | Parsed | TUI Display | Priority |
|---|---|---|---|---|
| EEG waveforms | BLE | ✅ | ✅ chart | — |
| PPG waveforms | BLE | ✅ | ✅ chart | — |
| Battery level | BLE | ✅ | ✅ header text | — |
| Accelerometer | BLE | ✅ | ⚠️ footer text only | Medium |
| Gyroscope | BLE | ✅ | ⚠️ footer text only | Medium |
| Device info (fw/hw/sn) | BLE (control JSON) | ✅ | ❌ not shown | High |
| Control responses | BLE (control JSON) | ✅ | ❌ not shown | High |
| Contact quality (horseshoe) | Computed from EEG | ❌ | ❌ | High |
| Eye blink detection | Computed from EEG | ❌ | ❌ | High |
| Jaw clench detection | Computed from EEG | ❌ | ❌ | High |
| Frequency band powers | Computed from EEG | ❌ | ❌ | Future |

---

## 4. Control Commands Reference

Commands are sent to the control characteristic as length-prefixed ASCII strings.

| Command | Description | Response |
|---|---|---|
| `v1` | Request device info | JSON with `fw`, `hw`, `sn`, etc. |
| `v4` | Request extended info (Athena) | JSON |
| `s` | Request status | JSON with `bp`, `ts`, etc. |
| `h` | Halt / pause streaming | `{"rc":0}` |
| `d` | Resume streaming (Classic) | `{"rc":0}` |
| `dc001` | Resume streaming (Athena) | `{"rc":0}` or `{"rc":69}` (unsupported) |
| `p21` | Preset: EEG only (Classic) | `{"rc":0}` |
| `p20` | Preset: EEG + AUX (Classic) | `{"rc":0}` |
| `p50` | Preset: EEG + PPG (Classic) | `{"rc":0}` |
| `p1045` | Preset: all sensors (Athena) | `{"rc":0}` |
| `L1` | Enable LED (Athena) | `{"rc":0}` |

---

## 5. References

- [muse-js](https://github.com/urish/muse-js) — TypeScript Classic protocol reference
- [muse-jsx](https://github.com/eugenehp/muse-jsx) — TypeScript Athena protocol reference
- [OpenMuse](https://github.com/DominiqueMakowski/OpenMuse) — Python Athena decoder
- [Mind Monitor](https://www.mind-monitor.com/Technical_Manual.php) — Commercial app with HSI, FFT, spectrogram
- [muse-lsl](https://github.com/alexandrebarachant/muse-lsl) — Python LSL streaming
- [Muse SDK docs (archived)](https://web.archive.org/web/20181115045420/http://developer.choosemuse.com/tools/available-data) — Official data reference
