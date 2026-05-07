#!/usr/bin/env python3
"""
stream_view.py — Binary camera stream viewer for ESP32-P4.

Protocol: [8-byte magic] + [W: uint16 LE] + [H: uint16 LE] + [W×H×2 bytes RGB565]

W and H are read from the stream header so the viewer auto-adapts to
VISION_RES_QVGA (80×60) or VISION_RES_HVGA (80×53) without any changes here.

Three panels: RGB565→RGB  |  RGB565→BGR  |  Luminance Y

In DETECTION_STREAM mode, text lines (M: POSE:) arrive between binary frames.
They are captured during magic sync and overlaid on the first panel.

Close the window or Ctrl-C to quit.

Usage:
    python3 stream_view.py [/dev/ttyACM0] [baud]
"""

import sys
import re
import struct
import time
from collections import deque
import serial
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

PORT  = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD  = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
MAGIC = bytes([0xAA, 0x55, 0xA5, 0x5A, 0xF0, 0x0F, 0x50, 0x3C])

print(f"Opening {PORT} @ {BAUD} baud — waiting for first frame …")
print("LEFT=RGB565→RGB  CENTRE=RGB565→BGR  RIGHT=luminance Y")
print("Close the window or Ctrl-C to quit.")

ser = serial.Serial(PORT, BAUD, timeout=3)

# ── POSE + TOF state (updated from UART text lines between frames) ───────────
_latest_pose_txt = ""   # cyan overlay on panel 1, empty when no fix
_latest_tof_txt  = ""   # yellow overlay on panel 3, empty until first reading

_POSE_RE = re.compile(
    r'POSE:(\d+):([-\d.]+):([-\d.]+):([-\d.]+)'
    r':([-\d.]+):([-\d.]+):([-\d.]+):([-\d.]+)'
)
# TOF:<dist0>,<dist1>,...mm  where each value is a number or ---
_TOF_RE = re.compile(r'TOF:([\d,\-]+)mm')

def _handle_text_line(raw: bytes) -> None:
    global _latest_pose_txt, _latest_tof_txt
    try:
        line = raw.decode('ascii', errors='replace').strip()
    except Exception:
        return
    if not line:
        return
    print(f"[uart] {line}", flush=True)
    m = _POSE_RE.search(line)
    if m:
        n  = int(m.group(1))
        x, y, z = float(m.group(2)), float(m.group(3)), float(m.group(4))
        _latest_pose_txt = f"POSE n={n}  x={x:.2f} y={y:.2f} z={z:.2f} m"
    elif line.startswith('M') and not m:
        _latest_pose_txt = line
    t = _TOF_RE.search(line)
    if t:
        parts = t.group(1).split(',')
        labels = []
        for i, p in enumerate(parts):
            if p == '---':
                labels.append(f"S{i}: --")
            else:
                try:
                    mm = int(p)
                    labels.append(f"S{i}: {mm} mm  ({mm/1000:.2f} m)")
                except ValueError:
                    labels.append(f"S{i}: ?")
        _latest_tof_txt = "ToF\n" + "\n".join(labels)

# ── helpers ───────────────────────────────────────────────────────────────────
def sync_to_magic():
    """Scan byte-by-byte until the 8-byte magic header is found.
    Printable ASCII bytes between binary frames are accumulated and parsed
    as text lines so POSE data is captured without a separate thread."""
    buf      = b''
    line_buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            return False
        buf = (buf + b)[-len(MAGIC):]
        if buf == MAGIC:
            return True
        c = b[0]
        if c == 0x0A:                   # newline — flush line
            if line_buf:
                _handle_text_line(bytes(line_buf))
            line_buf.clear()
        elif c == 0x0D:                 # CR — ignore
            pass
        elif 0x20 <= c < 0x80:         # printable ASCII
            line_buf.append(c)
            if len(line_buf) > 300:
                line_buf.clear()
        else:
            line_buf.clear()            # binary byte — reset line buffer

def read_frame():
    """Sync to magic, read W/H header, read pixels. Returns (W, H, raw) or None."""
    if not sync_to_magic():
        return None
    hdr = ser.read(4)
    if len(hdr) < 4:
        return None
    W, H = struct.unpack('<HH', hdr)
    if W == 0 or H == 0 or W > 640 or H > 480:
        return None
    raw = ser.read(W * H * 2)
    if len(raw) < W * H * 2:
        return None
    return W, H, raw

def decode_rgb565(raw_bytes, W, H, swap_rb=False):
    arr = np.frombuffer(raw_bytes, dtype='<u2').astype(np.uint32)
    if swap_rb:
        b5 = (arr >> 11) & 0x1F;  g6 = (arr >> 5) & 0x3F;  r5 = arr & 0x1F
    else:
        r5 = (arr >> 11) & 0x1F;  g6 = (arr >> 5) & 0x3F;  b5 = arr & 0x1F
    r8 = (r5 * 527 >> 6).clip(0, 255).astype(np.uint8)
    g8 = (g6 * 259 >> 6).clip(0, 255).astype(np.uint8)
    b8 = (b5 * 527 >> 6).clip(0, 255).astype(np.uint8)
    return np.stack([r8, g8, b8], axis=-1).reshape(H, W, 3)

# ── wait for first frame to learn dimensions ──────────────────────────────────
print("Syncing …")
first = None
while first is None:
    first = read_frame()
    if first is None:
        print("  timeout — retrying")
W, H, first_raw = first
print(f"Stream: {W}×{H}  ({W*H*2} bytes/frame payload)")

# ── matplotlib setup (sized to actual stream dimensions) ──────────────────────
fig, axes = plt.subplots(1, 3, figsize=(18, 5))
fig.patch.set_facecolor('#111')
titles = ["RGB565 → RGB  (standard)", "RGB565 → BGR  (swapped)", "Luminance Y"]
ims = []
for ax, title in zip(axes, titles):
    ax.set_facecolor('#111')
    ax.axis('off')
    ax.set_title(title, color='white', pad=6, fontsize=10)
    im = ax.imshow(np.zeros((H, W, 3), dtype=np.uint8),
                   interpolation='nearest', aspect='equal', vmin=0, vmax=255)
    ims.append(im)

stats_txt = axes[0].text(0.01, 0.02, "waiting…",
                          transform=axes[0].transAxes,
                          color='lime', fontsize=8, va='bottom',
                          family='monospace')

pose_txt = axes[0].text(0.01, 0.98, "",
                         transform=axes[0].transAxes,
                         color='cyan', fontsize=8, va='top',
                         family='monospace',
                         bbox=dict(facecolor='#000000aa', edgecolor='none', pad=2))

tof_txt = axes[2].text(0.01, 0.98, "",
                        transform=axes[2].transAxes,
                        color='yellow', fontsize=9, va='top',
                        family='monospace',
                        bbox=dict(facecolor='#000000cc', edgecolor='none', pad=3))

plt.tight_layout()
plt.ion()
plt.show()

frame_n = 0
_frame_ts = deque(maxlen=21)   # last 20 frame timestamps for rolling FPS

def display_frame(W, H, raw):
    global frame_n
    frame_n += 1
    _frame_ts.append(time.monotonic())
    fps = ((len(_frame_ts) - 1) / (_frame_ts[-1] - _frame_ts[0])
           if len(_frame_ts) >= 2 else 0.0)

    rgb = decode_rgb565(raw, W, H, swap_rb=False)
    bgr = decode_rgb565(raw, W, H, swap_rb=True)
    luma = (77  * rgb[:, :, 0].astype(np.uint32)
          + 150 * rgb[:, :, 1].astype(np.uint32)
          +  29 * rgb[:, :, 2].astype(np.uint32)) >> 8
    luma_rgb = np.stack([luma.clip(0, 255).astype(np.uint8)] * 3, axis=-1)

    max_val = int(rgb.max())
    if max_val > 0:
        s = min(255.0 / max_val, 32.0)
        rgb_d  = (rgb.astype(np.float32)      * s).clip(0, 255).astype(np.uint8)
        bgr_d  = (bgr.astype(np.float32)      * s).clip(0, 255).astype(np.uint8)
        luma_d = (luma_rgb.astype(np.float32) * s).clip(0, 255).astype(np.uint8)
    else:
        rgb_d, bgr_d, luma_d = rgb, bgr, luma_rgb

    ims[0].set_data(rgb_d)
    ims[1].set_data(bgr_d)
    ims[2].set_data(luma_d)

    r_mean = rgb[:, :, 0].mean()
    g_mean = rgb[:, :, 1].mean()
    b_mean = rgb[:, :, 2].mean()
    status = '⚠ DARK' if max_val < 20 else ('⚠ BRIGHT' if max_val > 250 else '✓ OK')
    stats_txt.set_text(
        f"frame #{frame_n}  {fps:.1f} fps  {W}×{H}   "
        f"R={r_mean:.0f} G={g_mean:.0f} B={b_mean:.0f}  max={max_val}  {status}"
    )
    pose_txt.set_text(_latest_pose_txt)
    tof_txt.set_text(_latest_tof_txt)

    fig.canvas.draw()
    fig.canvas.flush_events()

# display the first frame already read during sync
display_frame(W, H, first_raw)

# ── main loop ─────────────────────────────────────────────────────────────────
try:
    while plt.fignum_exists(fig.number):
        result = read_frame()
        if result is None:
            print("timeout or bad frame — retrying")
            continue
        fw, fh, raw = result
        if (fw, fh) != (W, H):
            print(f"dimensions changed {W}×{H} → {fw}×{fh} — restart viewer to reinit")
            continue
        display_frame(fw, fh, raw)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Done.")
