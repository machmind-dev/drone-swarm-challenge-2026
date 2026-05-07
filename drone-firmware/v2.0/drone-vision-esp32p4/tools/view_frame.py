#!/usr/bin/env python3
"""
view_frame.py — Live viewer for ESP32-P4 ArUco camera debug frames.

Shows TWO panels:
  Left  — Color RGB (what the camera actually captures)
  Right — Grayscale (what OpenCV/ArUco processes)

Protocol:
    FSTART:80x60          → grayscale frame (4800 bytes → 9600 hex chars)
    FSTART:80x60:RGB      → color frame (14400 bytes → 28800 hex chars)
    FEND

Usage:
    python3 view_frame.py [/dev/ttyACM0] [baud]
"""

import sys
import serial
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
W, H = 80, 60

print(f"Opening {PORT} @ {BAUD} baud — waiting for first frame...")
print("Left panel = color.  Right panel = grayscale.")
print("Close the window or Ctrl-C to quit.")

ser = serial.Serial(PORT, BAUD, timeout=5)

# --- two-panel matplotlib setup -----------------------------------------------
fig, (ax_color, ax_gray) = plt.subplots(1, 2, figsize=(14, 6))
fig.patch.set_facecolor('#1a1a1a')
for ax in (ax_color, ax_gray):
    ax.set_facecolor('#1a1a1a')
    ax.axis('off')

color_data = np.zeros((H, W, 3), dtype=np.uint8)
gray_data  = np.zeros((H, W),    dtype=np.uint8)

im_color = ax_color.imshow(color_data, interpolation='bilinear', aspect='equal')
im_gray  = ax_gray.imshow(gray_data, cmap='gray', vmin=0, vmax=255,
                           interpolation='bilinear', aspect='equal')

ax_color.set_title("Camera (color RGB)", color='white', pad=6)
ax_gray.set_title("ArUco input (grayscale + normalised)", color='white', pad=6)

stats_color = ax_color.text(0.01, 0.02, "waiting…", transform=ax_color.transAxes,
                             color='lime', fontsize=8, va='bottom', family='monospace')
stats_gray  = ax_gray.text(0.01, 0.02, "waiting…", transform=ax_gray.transAxes,
                            color='lime', fontsize=8, va='bottom', family='monospace')
plt.tight_layout()
plt.ion()
plt.show()

frame_counts = [0, 0]   # [color, gray]


def read_frame():
    """Block until a complete FSTART…FEND block; return (is_rgb, array) or None."""
    in_frame = False
    is_rgb   = False
    hex_buf  = []

    while True:
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            return None

        if not line:
            continue

        if line.startswith("FSTART:"):
            in_frame = True
            is_rgb   = line.endswith(":RGB")
            hex_buf  = []
            continue

        if line == "FEND":
            if in_frame and hex_buf:
                hex_str = "".join(hex_buf)
                expected = W * H * (3 if is_rgb else 1) * 2
                if len(hex_str) == expected:
                    try:
                        pixels = bytes.fromhex(hex_str)
                        if is_rgb:
                            arr = np.frombuffer(pixels, dtype=np.uint8).reshape(H, W, 3)
                        else:
                            arr = np.frombuffer(pixels, dtype=np.uint8).reshape(H, W)
                        return (is_rgb, arr)
                    except ValueError:
                        print(f"Hex decode error — skipping")
                else:
                    print(f"Length mismatch: got {len(hex_str)}, expected {expected} (rows={len(hex_buf)})")
            in_frame = False
            hex_buf  = []
            continue

        if in_frame:
            hex_buf.append(line)
        else:
            print(line)


# --- simple blocking loop — avoids FuncAnimation timer queuing ----------------
try:
    while plt.fignum_exists(fig.number):
        result = read_frame()
        if result is None:
            break

        is_rgb, arr = result

        if is_rgb:
            frame_counts[0] += 1
            max_val = int(arr.max())
            mean_r = arr[:, :, 0].mean()
            mean_g = arr[:, :, 1].mean()
            mean_b = arr[:, :, 2].mean()
            # Per-channel stretch: each R/G/B channel independently spans 0-255.
            # This removes ISP white-balance bias (OV5647 raw output is green-heavy)
            # so colour looks natural even in dark scenes.
            arr_f = arr.astype(np.float32)
            arr_display = np.zeros_like(arr)
            for ch in range(3):
                ch_max = arr_f[:, :, ch].max()
                if ch_max > 4:          # skip channels that are essentially noise
                    arr_display[:, :, ch] = (arr_f[:, :, ch] * (255.0 / ch_max)).clip(0, 255).astype(np.uint8)
            status = '⚠ DARK' if max_val < 20 else ('⚠ BRIGHT' if max_val > 250 else '✓ OK')
            im_color.set_data(arr_display)
            stats_color.set_text(
                f"color #{frame_counts[0]}  R={mean_r:.0f} G={mean_g:.0f} B={mean_b:.0f}  max={max_val}  {status}"
            )
        else:
            frame_counts[1] += 1
            mean_val = arr.mean()
            max_val  = int(arr.max())
            status = '⚠ DARK' if mean_val < 20 else ('⚠ BRIGHT' if mean_val > 230 else '✓ OK')
            im_gray.set_data(arr)
            stats_gray.set_text(
                f"gray #{frame_counts[1]}   mean={mean_val:.1f}  max={max_val}  {status}"
            )
            if max_val > 0:
                im_gray.set_clim(vmin=0, vmax=max(max_val, 32))

        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Done.")
