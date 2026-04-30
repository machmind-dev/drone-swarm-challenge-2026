#!/usr/bin/env python3
"""
stream_view.py — Binary camera stream viewer for ESP32-P4 CAMERA_VIEW_MODE.

Protocol: [0xAA][0x55][0xA5][0x5A][0xF0][0x0F][0x50][0x3C]  8-byte magic
          + 80×60×2 bytes raw RGB565 (little-endian)

8-byte magic prevents false-sync on RGB565 image data that happened to
contain the old 4-byte magic as pixel values.

Shows THREE panels:
  Left   — RGB565 decoded as RGB  (standard V4L2 interpretation)
  Centre — RGB565 decoded as BGR  (byte-swapped interpretation)
  Right  — Luminance (Y channel)

If one colour panel looks correct, that tells us the ISP byte order.
Press Q to quit.

Usage:
    python3 stream_view.py [/dev/ttyACM0] [baud]
"""

import sys
import struct
import serial
import numpy as np
import matplotlib
matplotlib.use('TkAgg')          # force non-blocking backend
import matplotlib.pyplot as plt

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = int(sys.argv[2])         if len(sys.argv) > 2 else 921600
W, H = 80, 64                   # 800x640 sensor → 80x64 keeps 5:4 aspect ratio
FRAME_BYTES = W * H * 2         # RGB565 = 2 bytes/pixel
MAGIC = bytes([0xAA, 0x55, 0xA5, 0x5A, 0xF0, 0x0F, 0x50, 0x3C])

print(f"Opening {PORT} @ {BAUD} baud — waiting for first frame …")
print("LEFT=RGB565→RGB  CENTRE=RGB565→BGR  RIGHT=luminance Y")
print("Close the window or Ctrl-C to quit.")

ser = serial.Serial(PORT, BAUD, timeout=3)

# ── matplotlib setup ──────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 3, figsize=(18, 5))
fig.patch.set_facecolor('#111')
titles = ["RGB565 → RGB  (standard)", "RGB565 → BGR  (swapped)", "Luminance Y"]
ims = []
for ax, title in zip(axes, titles):
    ax.set_facecolor('#111')
    ax.axis('off')
    ax.set_title(title, color='white', pad=6, fontsize=10)
    dummy = np.zeros((H, W, 3), dtype=np.uint8)
    im = ax.imshow(dummy, interpolation='nearest', aspect='equal',
                   vmin=0, vmax=255)
    ims.append(im)

stats_txt = axes[0].text(0.01, 0.02, "waiting…",
                          transform=axes[0].transAxes,
                          color='lime', fontsize=8, va='bottom',
                          family='monospace')
plt.tight_layout()
plt.ion()
plt.show()

frame_n = [0]

# ── sync helper ───────────────────────────────────────────────────────────────
def sync_to_magic():
    """Scan serial bytes until the magic header is found."""
    buf = b''
    while True:
        b = ser.read(1)
        if not b:
            return False          # timeout
        buf = (buf + b)[-len(MAGIC):]
        if buf == MAGIC:
            return True

# ── decode RGB565 → RGB888 numpy array ───────────────────────────────────────
def decode_rgb565(raw_bytes, swap_rb=False):
    arr = np.frombuffer(raw_bytes, dtype='<u2').astype(np.uint32)
    if swap_rb:
        b5 = (arr >> 11) & 0x1F
        g6 = (arr >>  5) & 0x3F
        r5 =  arr        & 0x1F
    else:
        r5 = (arr >> 11) & 0x1F
        g6 = (arr >>  5) & 0x3F
        b5 =  arr        & 0x1F
    r8 = (r5 * 527 >> 6).clip(0, 255).astype(np.uint8)
    g8 = (g6 * 259 >> 6).clip(0, 255).astype(np.uint8)
    b8 = (b5 * 527 >> 6).clip(0, 255).astype(np.uint8)
    return np.stack([r8, g8, b8], axis=-1).reshape(H, W, 3)

# ── main loop ────────────────────────────────────────────────────────────────
try:
    while plt.fignum_exists(fig.number):
        if not sync_to_magic():
            print("timeout waiting for magic header — retrying")
            continue

        raw = ser.read(FRAME_BYTES)
        if len(raw) < FRAME_BYTES:
            print(f"short read: {len(raw)}/{FRAME_BYTES} bytes — skipping")
            continue

        frame_n[0] += 1

        rgb = decode_rgb565(raw, swap_rb=False)
        bgr = decode_rgb565(raw, swap_rb=True)

        # Luminance (BT.601) from the RGB panel
        luma = (77 * rgb[:,:,0].astype(np.uint32)
              + 150 * rgb[:,:,1].astype(np.uint32)
              +  29 * rgb[:,:,2].astype(np.uint32)) >> 8
        luma = luma.clip(0, 255).astype(np.uint8)
        luma_rgb = np.stack([luma, luma, luma], axis=-1)

        # Auto-stretch: scale so brightest pixel = 255 (up to 32x gain)
        max_val = int(rgb.max())
        if max_val > 0:
            scale = min(255.0 / max_val, 32.0)
            rgb_d = (rgb.astype(np.float32) * scale).clip(0, 255).astype(np.uint8)
            bgr_d = (bgr.astype(np.float32) * scale).clip(0, 255).astype(np.uint8)
            luma_d = (luma_rgb.astype(np.float32) * scale).clip(0, 255).astype(np.uint8)
        else:
            rgb_d, bgr_d, luma_d = rgb, bgr, luma_rgb

        ims[0].set_data(rgb_d)
        ims[1].set_data(bgr_d)
        ims[2].set_data(luma_d)

        r_mean = rgb[:,:,0].mean()
        g_mean = rgb[:,:,1].mean()
        b_mean = rgb[:,:,2].mean()
        status = '⚠ DARK' if max_val < 20 else ('⚠ BRIGHT' if max_val > 250 else '✓ OK')
        stats_txt.set_text(
            f"frame #{frame_n[0]}   R={r_mean:.0f} G={g_mean:.0f} B={b_mean:.0f}  max={max_val}  {status}"
        )

        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Done.")
