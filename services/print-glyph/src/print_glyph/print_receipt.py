"""
Magic Carpet Ride — Glyph Receipt Printer
Syncopy Design Lab / Bombay Beach Biennale

Prints a personalized receipt showing one glyph per dimension
based on each participant's EEG results.

USAGE:
  python -m print_glyph.print_receipt \
    --absorption deep --attunement porous \
    --unknown lean_in --witnessed withdraw

Preview without printing:
  python -m print_glyph.print_receipt \
    --absorption deep --attunement porous \
    --unknown lean_in --witnessed withdraw \
    --preview-only

VALID VALUES:
  absorption:  "deep" | "surface"
  attunement:  "porous" | "boundaried"
  unknown:     "lean_in" | "hold_back"
  witnessed:   "approach" | "withdraw"
"""

import argparse
import contextlib
import io
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# Printer USB IDs (Rongta 80mm)
VENDOR_ID = 0x0FE6
PRODUCT_ID = 0x811E

# Receipt dimensions
W = 576  # 80mm at 203dpi
BG = 255  # white
FG = 0  # black

DIR = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------------
# Drawing helpers
# ---------------------------------------------------------------------------


def spacer(h=12):
  return Image.new("L", (W, h), BG)


def hline(t=2):
  return Image.new("L", (W, t), FG)


def hline_light(t=1):
  return Image.new("L", (W, t), 180)


def _load_font(size):
  font_paths = [
    "/Library/Fonts/Arial.ttf",
    "/System/Library/Fonts/Helvetica.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
  ]
  for path in font_paths:
    if os.path.exists(path):
      try:
        return ImageFont.truetype(path, size)
      except Exception:
        continue
  return ImageFont.load_default()


def label_img(text, size=18, color=FG):
  font = _load_font(size)
  dummy = Image.new("L", (1, 1))
  bbox = ImageDraw.Draw(dummy).textbbox((0, 0), text, font=font)
  tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
  img = Image.new("L", (W, th + 22), BG)
  ImageDraw.Draw(img).text(((W - tw) // 2, 8), text, font=font, fill=color)
  return img


def dim_label(text):
  return label_img(text, size=16, color=80)


def result_label(text):
  return label_img(text, size=22, color=FG)


# ---------------------------------------------------------------------------
# Wave image loading
# ---------------------------------------------------------------------------

DEEP_IMG = os.path.join(DIR, "wave_deep.png")
SURF_IMG = os.path.join(DIR, "wave_surf.png")
EYE_OPEN_IMG = os.path.join(DIR, "eye_open.png")
EYE_CLOSED_IMG = os.path.join(DIR, "eye_closed.png")


def scale_wave_auto_center(wave_img, target_w, target_h):
  """Crop to content, scale to fit target, and center on white canvas."""
  wave = wave_img.convert("L")
  arr = np.array(wave)
  col_idx = np.where(np.any(arr < 200, axis=0))[0]
  row_idx = np.where(np.any(arr < 200, axis=1))[0]
  pad = 16
  wave = wave.crop(
    (
      max(0, col_idx.min() - pad),
      max(0, row_idx.min() - pad),
      min(wave.width, col_idx.max() + pad),
      min(wave.height, row_idx.max() + pad),
    )
  )
  ratio = min(target_w / wave.width, target_h / wave.height)
  new_w, new_h = int(wave.width * ratio), int(wave.height * ratio)
  wave = wave.resize((new_w, new_h), Image.LANCZOS)
  canvas = Image.new("L", (target_w, target_h), BG)
  canvas.paste(wave, ((target_w - new_w) // 2, (target_h - new_h) // 2))
  return canvas


# ---------------------------------------------------------------------------
# Glyph drawing functions
# ---------------------------------------------------------------------------


def draw_vesica(filled_center=True):
  """Two overlapping circles — porous fills the intersection, boundaried fills the outer."""
  img = Image.new("L", (W, 130), BG)
  draw = ImageDraw.Draw(img)
  cx, cy, r, offset = W // 2, 65, 48, 26
  lx, rx = cx - offset, cx + offset

  if filled_center:
    # Fill the vesica (intersection)
    for y in range(cy - r, cy + r):
      dy = y - cy
      if r**2 >= dy**2:
        x1_l = lx + int((r**2 - dy**2) ** 0.5)
        x1_r = rx - int((r**2 - dy**2) ** 0.5)
        if x1_r <= x1_l:
          draw.line([(x1_r, y), (x1_l, y)], fill=FG)
    draw.ellipse([lx - r, cy - r, lx + r, cy + r], outline=FG, width=3)
    draw.ellipse([rx - r, cy - r, rx + r, cy + r], outline=FG, width=3)
  else:
    # Fill the outer lobes (boundaried)
    draw.ellipse([lx - r, cy - r, lx + r, cy + r], outline=FG, width=3)
    draw.ellipse([rx - r, cy - r, rx + r, cy + r], outline=FG, width=3)
    for y in range(cy - r, cy + r):
      dy = y - cy
      if r**2 >= dy**2:
        half_chord = int((r**2 - dy**2) ** 0.5)
        # Left lobe fill (left edge of left circle to right edge of right circle)
        draw.line([(lx - half_chord, y), (rx - half_chord, y)], fill=FG)
        draw.line([(lx + half_chord, y), (rx + half_chord, y)], fill=FG)
  return img


def draw_gate(open_door=True):
  """Gate/doorway — open for lean_in, closed for hold_back."""
  img = Image.new("L", (W, 120), BG)
  draw = ImageDraw.Draw(img)
  cx = W // 2
  gap, pw = 62, 16
  lp = cx - gap - pw
  rp = cx + gap

  # Pillars
  draw.rectangle([lp, 14, lp + pw, 94], fill=FG)
  draw.rectangle([rp, 14, rp + pw, 94], fill=FG)
  # Lintel
  draw.rectangle([lp - 4, 10, rp + pw + 4, 22], fill=FG)

  if open_door:
    # Doors ajar — partial fill on each side
    draw.rectangle([lp + pw + 2, 24, cx - 14, 90], fill=FG)
    draw.polygon([(rp - 2, 24), (rp - 14, 28), (rp - 14, 90), (rp - 2, 91)], fill=FG)
  else:
    # Doors closed — full fill
    draw.rectangle([lp + pw + 2, 24, cx - 1, 90], fill=FG)
    draw.rectangle([cx + 1, 24, rp - 2, 90], fill=FG)
  return img


def draw_eye(open_eye=True):
  """Eye shape — open with pupil for approach, closed with lashes for withdraw."""
  img = Image.new("L", (W, 120), BG)
  draw = ImageDraw.Draw(img)
  cx, cy, ew, eh = W // 2, 48, 160, 44

  # Eye outline points (elliptical almond shape)
  pts_top = []
  pts_bot = []
  for i in range(ew + 1):
    norm = (i - ew // 2) / (ew // 2)
    bulge = int(eh // 2 * (1 - norm**2) ** 0.5) if abs(norm) <= 1 else 0
    pts_top.append((cx - ew // 2 + i, cy - bulge))
    pts_bot.append((cx - ew // 2 + i, cy + bulge))

  pts_bot_rev = list(reversed(pts_bot))
  draw.polygon(pts_top + pts_bot_rev, outline=FG, width=3, fill=BG)

  if open_eye:
    # Pupil — larger solid circle with specular highlight
    pr = 30
    draw.ellipse([cx - pr, cy - pr, cx + pr, cy + pr], fill=FG)
    # Highlight dot
    draw.ellipse([cx + 8, cy - 14, cx + 18, cy - 4], fill=BG)
  else:
    # Closed — fill upper half
    draw.polygon(pts_top + [(cx + ew // 2, cy), (cx - ew // 2, cy)], fill=FG)

    # Pupil bulge visible under the closed lid
    pupil_r = 18
    draw.ellipse(
      [cx - pupil_r, cy - pupil_r // 2, cx + pupil_r, cy + pupil_r],
      fill=FG,
    )

    # Lid line
    draw.line([(cx - ew // 2 + 4, cy), (cx + ew // 2 - 4, cy)], fill=FG, width=3)

    # Curved lashes — control point pushed sideways for visible arc
    lash_data = [
      # (start_x, start_y, end_x, end_y, curve_direction)
      (cx - ew // 2 + 4, cy, cx - ew // 2 - 16, cy + 28, -18),
      (cx - 35, cy + 2, cx - 48, cy + 35, -14),
      (cx, cy + 4, cx, cy + 38, 0),
      (cx + 35, cy + 2, cx + 48, cy + 35, 14),
      (cx + ew // 2 - 4, cy, cx + ew // 2 + 16, cy + 28, 18),
    ]
    for x1, y1, x2, y2, curve in lash_data:
      # Control point offset perpendicular to the line for a real curve
      mid_x = (x1 + x2) / 2 + curve
      mid_y = (y1 + y2) / 2 + 12
      steps = 50
      prev = None
      for t_i in range(steps + 1):
        tt = t_i / steps
        bx = (1 - tt) ** 2 * x1 + 2 * (1 - tt) * tt * mid_x + tt**2 * x2
        by = (1 - tt) ** 2 * y1 + 2 * (1 - tt) * tt * mid_y + tt**2 * y2
        if prev is not None:
          draw.line([prev, (bx, by)], fill=FG, width=4)
        prev = (bx, by)
  return img


# ---------------------------------------------------------------------------
# Glyph selectors per dimension
# ---------------------------------------------------------------------------


def absorption_glyph(result):
  for path in [DEEP_IMG, SURF_IMG]:
    if not os.path.exists(path):
      print(f"ERROR: Missing wave image: {path}")
      sys.exit(1)

  if result == "deep":
    deep_img = Image.open(DEEP_IMG).convert("L")
    return [scale_wave_auto_center(deep_img, W, 90)]  # , result_label("D E E P")]
  else:
    surf_img = Image.open(SURF_IMG).convert("L")
    return [scale_wave_auto_center(surf_img, W, 52)]  # , result_label("S U R F A C E")]


def attunement_glyph(result):
  if result == "porous":
    return [draw_vesica(filled_center=True)]  # , result_label("P O R O U S")]
  else:
    return [draw_vesica(filled_center=False)]  # , result_label("B O U N D A R I E D")]


def unknown_glyph(result):
  if result == "lean_in":
    return [draw_gate(open_door=True)]  # , result_label("L E A N   I N")]
  else:
    return [draw_gate(open_door=False)]  # , result_label("H O L D   B A C K")]


def _load_glyph_img(path, target_h=120):
  """Load a glyph PNG, ensure white background, scale and center."""
  img = Image.open(path).convert("RGBA")
  # Composite onto white background to remove any transparency
  bg = Image.new("RGBA", img.size, (255, 255, 255, 255))
  bg.paste(img, mask=img.split()[3])
  return scale_wave_auto_center(bg.convert("L"), W, target_h)


def witnessed_glyph(result):
  if result == "approach":
    return [_load_glyph_img(EYE_OPEN_IMG)]  # , result_label("A P P R O A C H")]
  else:
    return [_load_glyph_img(EYE_CLOSED_IMG)]  # , result_label("W I T H D R A W")]


# ---------------------------------------------------------------------------
# Build receipt image
# ---------------------------------------------------------------------------


def build_receipt(absorption, attunement, unknown, witnessed):
  parts = [spacer(16), hline(2), hline_light(), spacer(10)]

  # Absorption
  # parts.append(dim_label("A B S O R P T I O N"))
  parts.append(spacer(6))
  parts += absorption_glyph(absorption)
  parts.append(result_label("A B S O R P T I O N"))
  parts += [spacer(10), hline_light(), hline(2), spacer(10)]

  # Attunement
  # parts.append(dim_label("A T T U N E M E N T"))
  parts.append(spacer(6))
  parts += attunement_glyph(attunement)
  parts.append(result_label("A T T U N E M E N T"))
  parts += [spacer(10), hline_light(), hline(2), spacer(10)]

  # The Unknown
  # parts.append(dim_label("T H E   U N K N O W N"))
  parts.append(spacer(6))
  parts += unknown_glyph(unknown)
  parts.append(result_label("T H E   U N K N O W N"))
  parts += [spacer(10), hline_light(), hline(2), spacer(10)]

  # Being Witnessed
  # parts.append(dim_label("B E I N G   W I T N E S S E D"))
  parts.append(spacer(6))
  parts += witnessed_glyph(witnessed)
  parts.append(result_label("W I T N E S S E D"))
  parts += [spacer(10), hline_light(), hline(2), spacer(12)]

  # Footer
  parts.append(spacer(20))
  parts.append(label_img("MAGIC CARPET RIDE", size=20, color=40))
  parts.append(label_img("BOMBAY BEACH BIENNALE", size=20, color=40))
  parts.append(spacer(6))
  parts.append(label_img("SYNCOPY DESIGN LAB", size=20, color=40))
  parts.append(spacer(100))

  # Stitch all parts vertically
  total_h = sum(p.height for p in parts)
  receipt = Image.new("L", (W, total_h), BG)
  y = 0
  for p in parts:
    receipt.paste(p, (0, y))
    y += p.height
  return receipt


# ---------------------------------------------------------------------------
# Print
# ---------------------------------------------------------------------------


def print_receipt(absorption, attunement, unknown, witnessed, save_preview=True):
  valid = {
    "absorption": ["deep", "surface"],
    "attunement": ["porous", "boundaried"],
    "unknown": ["lean_in", "hold_back"],
    "witnessed": ["approach", "withdraw"],
  }
  values = [absorption, attunement, unknown, witnessed]
  for key, val in zip(valid.keys(), values):
    if val not in valid[key]:
      print(f"ERROR: Invalid value '{val}' for {key}. Must be one of: {valid[key]}")
      sys.exit(1)

  print(f"Building receipt: {absorption} / {attunement} / {unknown} / {witnessed}")
  receipt = build_receipt(absorption, attunement, unknown, witnessed)

  if save_preview:
    path = os.path.join(DIR, "receipt_preview.png")
    receipt.save(path)
    print(f"Preview saved: {path}")

  print("Connecting to printer...")
  try:
    from escpos.printer import Usb

    p = Usb(VENDOR_ID, PRODUCT_ID, 0, profile="default")
    print("Printing...")
    with contextlib.redirect_stdout(io.StringIO()):
      p.image(receipt, impl="bitImageRaster")
    p.cut()
    print("Done. Receipt cut.")
  except ImportError:
    print("ERROR: python-escpos not installed. Run: uv sync")
    sys.exit(1)
  except Exception as e:
    print(f"ERROR: Could not connect to printer.\n{e}")
    print("\nTroubleshooting:")
    print("  1. Make sure printer is plugged in and powered on")
    print("  2. On Mac install libusb: brew install libusb")
    print("  3. Check VENDOR_ID / PRODUCT_ID at top of script match your printer")
    sys.exit(1)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Print a Magic Carpet Ride glyph receipt.")
  parser.add_argument("--absorption", required=True, choices=["deep", "surface"])
  parser.add_argument("--attunement", required=True, choices=["porous", "boundaried"])
  parser.add_argument("--unknown", required=True, choices=["lean_in", "hold_back"])
  parser.add_argument("--witnessed", required=True, choices=["approach", "withdraw"])
  parser.add_argument(
    "--preview-only",
    action="store_true",
    help="Save a preview PNG without sending to printer",
  )
  args = parser.parse_args()

  if args.preview_only:
    receipt = build_receipt(args.absorption, args.attunement, args.unknown, args.witnessed)
    path = os.path.join(DIR, "receipt_preview.png")
    receipt.save(path)
    print(f"Preview saved: {path}")
  else:
    print_receipt(args.absorption, args.attunement, args.unknown, args.witnessed)
