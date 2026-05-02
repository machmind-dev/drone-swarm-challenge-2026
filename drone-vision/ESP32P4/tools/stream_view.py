#!/usr/bin/env python3
"""
stream_view.py — ESP32-P4 camera / detection stream viewer.

Supports two firmware modes on the same binary protocol:

  CAMERA_VIEW_MODE   — pure binary frames, no text
  DETECTION_STREAM   — binary frames interleaved with text lines (M: POSE:)

Binary protocol per frame:
  8 bytes  magic   [0xAA][0x55][0xA5][0x5A][0xF0][0x0F][0x50][0x3C]
  4 bytes  header  W (uint16 LE), H (uint16 LE)
  W×H×2 bytes      RGB565 pixels

In DETECTION_STREAM mode text lines arrive between frames.  The viewer
captures them, parses POSE: data, and overlays it on the image.

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
print("Close the window or Ctrl-C to quit.")

ser = serial.Serial(PORT, BAUD, timeout=3)

# ── latest state from UART text lines ────────────────────────────────────────
_latest_markers = ""   # e.g. "M7:2.34m M9:3.10m"
_latest_pose    = None  # dict(n, x, y, z, qx, qy, qz, qw) or None

_POSE_RE = re.compile(
    r'POSE:(\d+):([-\d.]+):([-\d.]+):([-\d.]+)'
    r':([-\d.]+):([-\d.]+):([-\d.]+):([-\d.]+)'
)

def _handle_text_line(raw: bytes) -> None:
    global _latest_markers, _latest_pose
    try:
        line = raw.decode('ascii', errors='replace').strip()
    except Exception:
        return
    if not line:
        return
    print(f"[uart] {line}", flush=True)

    m = _POSE_RE.search(line)
    if m:
        _latest_pose = dict(
            n=int(m.group(1)),
            x=float(m.group(2)), y=float(m.group(3)), z=float(m.group(4)),
            qx=float(m.group(5)), qy=float(m.group(6)),
            qz=float(m.group(7)), qw=float(m.group(8)),
        )
        # Strip POSE token — keep the marker-distance prefix
        _latest_markers = line[:m.start()].strip()
    elif line.startswith('M'):
        _latest_markers = line
        _latest_pose = None

# ── serial helpers ────────────────────────────────────────────────────────────
def sync_to_magic() -> bool:
    """Scan byte-by-byte until the 8-byte magic is found.
    Printable ASCII bytes accumulated between binary frames are parsed
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
        if c == 0x0A:                    # newline — flush text line
            if line_buf:
                _handle_text_line(bytes(line_buf))
            line_buf.clear()
        elif c == 0x0D:                  # CR — ignore
            pass
        elif 0x20 <= c < 0x80:          # printable ASCII
            line_buf.append(c)
            if len(line_buf) > 300:     # guard against runaway binary data
                line_buf.clear()
        else:
            line_buf.clear()            # non-printable binary byte — reset


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


def _decode_rgb565(raw_bytes: bytes, W: int, H: int) -> np.ndarray:
    arr = np.frombuffer(raw_bytes, dtype='<u2').astype(np.uint32)
    r5 = (arr >> 11) & 0x1F
    g6 = (arr >>  5) & 0x3F
    b5 =  arr        & 0x1F
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

# ── matplotlib setup ──────────────────────────────────────────────────────────
fig, ax = plt.subplots(1, 1, figsize=(10, 7))
fig.patch.set_facecolor('#111')
ax.set_facecolor('#111')
ax.axis('off')
ax.set_title("ESP32-P4  ArUco detection stream", color='white', pad=8, fontsize=12)

im = ax.imshow(np.zeros((H, W, 3), dtype=np.uint8),
               interpolation='nearest', aspect='equal', vmin=0, vmax=255)

# Status line — top-left (frame / fps / dimensions)
_status_txt = ax.text(
    0.01, 0.98, "waiting…",
    transform=ax.transAxes,
    color='lime', fontsize=9, va='top', family='monospace',
    bbox=dict(facecolor='#000000aa', edgecolor='none', pad=2),
)

# Marker distances — bottom-left
_marker_txt = ax.text(
    0.01, 0.02, "",
    transform=ax.transAxes,
    color='yellow', fontsize=9, va='bottom', family='monospace',
    bbox=dict(facecolor='#000000aa', edgecolor='none', pad=2),
)

# POSE overlay — bottom-right
_pose_txt = ax.text(
    0.99, 0.02, "",
    transform=ax.transAxes,
    color='cyan', fontsize=9, va='bottom', ha='right', family='monospace',
    bbox=dict(facecolor='#000000aa', edgecolor='none', pad=2),
)

plt.tight_layout()
plt.ion()
plt.show()

_frame_n  = 0
_frame_ts: deque = deque(maxlen=21)


def display_frame(W: int, H: int, raw: bytes) -> None:
    global _frame_n
    _frame_n += 1
    _frame_ts.append(time.monotonic())
    fps = ((len(_frame_ts) - 1) / (_frame_ts[-1] - _frame_ts[0])
           if len(_frame_ts) >= 2 else 0.0)

    rgb = _decode_rgb565(raw, W, H)
    im.set_data(rgb)

    _status_txt.set_text(f"frame #{_frame_n}  {fps:.1f} fps  {W}×{H}")
    _marker_txt.set_text(_latest_markers)

    if _latest_pose:
        p = _latest_pose
        _pose_txt.set_text(
            f"POSE  n={p['n']}\n"
            f"x={p['x']:6.2f}  y={p['y']:6.2f}  z={p['z']:6.2f} m\n"
            f"q=({p['qx']:.3f}, {p['qy']:.3f}, {p['qz']:.3f}, {p['qw']:.3f})"
        )
    else:
        _pose_txt.set_text("")

    fig.canvas.draw()
    fig.canvas.flush_events()


# display the first frame captured during sync
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
