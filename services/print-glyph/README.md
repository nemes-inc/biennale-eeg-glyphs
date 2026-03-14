# print-glyph

Thermal receipt printer for Magic Carpet Ride glyph readings. Generates a 576px-wide receipt image with one glyph per EEG dimension, prints on a Rongta 80mm printer via USB.

## Setup

Requires Python 3.11+. Uses [uv](https://docs.astral.sh/uv/) for dependency management.

```bash
cd services/print-glyph
uv venv
source .venv/bin/activate
uv sync
```

Mac users need libusb for USB printer access:

```bash
brew install libusb
```

## Usage

```bash
python -m print_glyph.print_receipt \
  --absorption deep --attunement porous \
  --unknown lean_in --witnessed withdraw
```

`--preview-only` saves `receipt_preview.png` without printing:

```bash
python -m print_glyph.print_receipt \
  --absorption deep --attunement porous \
  --unknown lean_in --witnessed withdraw \
  --preview-only
```

## EEG Dimensions

| Dimension | Values | Glyph |
|---|---|---|
| `--absorption` | `deep`, `surface` | Wave pattern (PNG asset) |
| `--attunement` | `porous`, `boundaried` | Vesica piscis (filled center vs outer lobes) |
| `--unknown` | `lean_in`, `hold_back` | Gate (open vs closed door) |
| `--witnessed` | `approach`, `withdraw` | Eye (open vs closed) |

## Printer Config

Rongta 80mm thermal printer. USB IDs hardcoded in `print_receipt.py`:

- Vendor: `0x0FE6`
- Product: `0x811E`

Update these if your printer differs. Find your IDs with `lsusb` or `system_profiler SPUSBDataType` on Mac.
