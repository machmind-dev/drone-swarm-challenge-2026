#!/usr/bin/env python3
"""
arena_view.py — Top-down arena map viewer for Mach Mind drone swarm.

Reads POSE lines from UART and shows a live top-down map:
  • Arena boundary (20 × 10 m)
  • ArUco marker positions (from aruco_pose.cpp world map)
  • Drone position + yaw heading (triangle pointing forward)
  • Fading position trail (last 200 fixes)
  • Status overlay: x / y / z / yaw / visible markers

Run standalone — uses the same serial port as stream_view.py.
Do NOT run both scripts at the same time (they would fight over the port).

Usage:
    python3 arena_view.py [/dev/ttyACM0] [baud]
"""

import sys
import re
import math
import time
import signal
import atexit
from collections import deque

import numpy as np
import serial
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Config ────────────────────────────────────────────────────────────────────
PORT      = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD      = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
TRAIL_LEN = 200     # number of past drone positions kept
POSE_TIMEOUT = 2.0  # seconds before "no fix"
DRAW_HZ   = 20      # target display refresh rate

# ── Arena & marker world map (from aruco_pose.cpp) ────────────────────────────
ARENA_W = 20.0
ARENA_H = 10.0

# (id, x_m, y_m, z_m)  — z shown in label, not in top-down plane
MARKERS = [
    ( 1, 20.0,  5.0, 4.0),  ( 9, 20.0,  5.0, 2.0),   # x=20 end pole
    ( 5,  0.0,  5.0, 4.0),  (13,  0.0,  5.0, 2.0),   # x=0  end pole
    ( 6,  5.0, 10.0, 4.0),  (14,  5.0, 10.0, 2.0),   # y=10 wall
    ( 7, 10.0, 10.0, 4.0),  (15, 10.0, 10.0, 2.0),
    ( 8, 15.0, 10.0, 4.0),  (16, 15.0, 10.0, 2.0),
    ( 4,  5.0,  0.0, 4.0),  (12,  5.0,  0.0, 2.0),   # y=0  wall
    ( 3, 10.0,  0.0, 4.0),  (11, 10.0,  0.0, 2.0),
    ( 2, 15.0,  0.0, 4.0),  (10, 15.0,  0.0, 2.0),
    (22, 10.0,  5.0, 2.0),                             # test marker (centre)
]

# ── Serial ────────────────────────────────────────────────────────────────────
print(f"Opening {PORT} @ {BAUD} baud — waiting for POSE lines …")
ser = serial.Serial(PORT, BAUD, timeout=0.05)
atexit.register(ser.close)
signal.signal(signal.SIGTERM, lambda _s, _f: sys.exit(0))
signal.signal(signal.SIGHUP,  lambda _s, _f: sys.exit(0))

# ── POSE / marker parsing ──────────────────────────────────────────────────────
_POSE_RE = re.compile(
    r'POSE:(\d+):([-\d.]+):([-\d.]+):([-\d.]+)'
    r':([-\d.]+):([-\d.]+):([-\d.]+):([-\d.]+)'
)
_MARKER_RE = re.compile(r'M(\d+):([\d.]+)m')

pose_x   = pose_y = pose_z = pose_yaw = 0.0
pose_n   = 0
pose_time = 0.0
visible  = []   # [(id_str, dist_str), ...]
trail    = deque(maxlen=TRAIL_LEN)


def _quat_to_yaw(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny, cosy))


def _parse_line(line):
    global pose_x, pose_y, pose_z, pose_yaw, pose_n, pose_time, visible
    m = _POSE_RE.search(line)
    if not m:
        return
    pose_n = int(m.group(1))
    pose_x = float(m.group(2))
    pose_y = float(m.group(3))
    pose_z = float(m.group(4))
    qx, qy, qz, qw = (float(m.group(i)) for i in range(5, 9))
    pose_yaw  = _quat_to_yaw(qx, qy, qz, qw)
    pose_time = time.monotonic()
    visible   = _MARKER_RE.findall(line)
    trail.append((pose_x, pose_y))


# Non-blocking byte-level parser — tolerates interleaved binary camera frames
_line_buf = bytearray()


def _poll_serial():
    global _line_buf
    try:
        waiting = ser.in_waiting
        data = ser.read(waiting if waiting > 0 else 1)
    except Exception:
        return
    for c in data:
        if c == 0x0A:                      # newline — flush
            if _line_buf:
                try:
                    _parse_line(_line_buf.decode('ascii', errors='replace'))
                except Exception:
                    pass
            _line_buf.clear()
        elif c == 0x0D:                    # CR — ignore
            pass
        elif 0x20 <= c < 0x80:            # printable ASCII
            _line_buf.append(c)
            if len(_line_buf) > 400:
                _line_buf.clear()
        else:
            _line_buf.clear()             # binary byte — discard partial line


# ── Matplotlib figure ─────────────────────────────────────────────────────────
PAD = 1.5   # metres of padding around the arena
fig, ax = plt.subplots(figsize=(14, 8))
fig.patch.set_facecolor('#0a0a1a')
ax.set_facecolor('#0d1117')
ax.set_aspect('equal')
ax.set_xlim(-PAD, ARENA_W + PAD)
ax.set_ylim(-PAD, ARENA_H + PAD)
ax.set_xlabel('X  (m)', color='#aaa', fontsize=10)
ax.set_ylabel('Y  (m)', color='#aaa', fontsize=10)
ax.tick_params(colors='#666')
for sp in ax.spines.values():
    sp.set_edgecolor('#333')
ax.grid(True, color='#1a1a2e', linewidth=0.8, linestyle='--')
ax.set_title('Arena — top-down view', color='white', pad=10, fontsize=12)

# Arena floor
ax.add_patch(mpatches.Rectangle(
    (0, 0), ARENA_W, ARENA_H,
    linewidth=2, edgecolor='#3366cc', facecolor='#0d1a2e', zorder=1))

# Axis labels inside arena edges
for x_pos in [0, 5, 10, 15, 20]:
    ax.axvline(x=x_pos, color='#1a2a4a', linewidth=0.5, zorder=1)
    ax.text(x_pos, -0.6, str(x_pos), color='#555', ha='center',
            fontsize=8, family='monospace')
for y_pos in [0, 5, 10]:
    ax.axhline(y=y_pos, color='#1a2a4a', linewidth=0.5, zorder=1)
    ax.text(-0.8, y_pos, str(y_pos), color='#555', va='center',
            fontsize=8, family='monospace')

# ── ArUco markers ─────────────────────────────────────────────────────────────
# Group by (x, y): multiple heights share one map position
_xy_groups: dict = {}
for mid, mx, my, mz in MARKERS:
    _xy_groups.setdefault((mx, my), []).append((mid, mz))

for (mx, my), entries in _xy_groups.items():
    # Sort by z descending: high first
    entries.sort(key=lambda e: -e[1])
    label = '/'.join(str(e[0]) for e in entries)
    heights = ', '.join(f"{e[1]:.0f}m" for e in entries)

    # High marker (z ≥ 3): filled gold square; Low: hollow
    for mid, mz in entries:
        color  = '#ffcc00' if mz >= 3.0 else '#cc8800'
        fill   = color     if mz >= 3.0 else 'none'
        ax.plot(mx, my, 's', markersize=11, color=color,
                markerfacecolor=fill, markeredgewidth=2, zorder=3)

    ax.text(mx, my + 0.55, f'M{label}', color='#ffdd66',
            fontsize=7.5, ha='center', va='bottom',
            family='monospace', zorder=4,
            bbox=dict(facecolor='#00000099', edgecolor='none', pad=1))

# ── Trail scatter plot ────────────────────────────────────────────────────────
trail_sc = ax.scatter([], [], s=12, c=[], cmap='cool',
                      vmin=0.0, vmax=1.0, alpha=0.55, zorder=5)

# ── Drone patch (triangle) ────────────────────────────────────────────────────
# Will be recreated each frame; _drone_patch holds current handle.
_drone_patch = [None]
_heading_line, = ax.plot([], [], '-', color='#00ffee',
                         linewidth=2.0, alpha=0.8, zorder=8)

DRONE_L = 0.9    # metres — nose-to-base length (visible at arena scale)
DRONE_W = 0.5    # metres — half-width of base

def _update_drone(x, y, yaw_deg):
    r = math.radians(yaw_deg + 90.0)   # firmware 0°=+Y wall; trig 0°=+X
    cr, sr = math.cos(r), math.sin(r)

    def _rot(px, py):
        return x + cr * px - sr * py, y + sr * px + cr * py

    nose = _rot(DRONE_L * 0.65, 0.0)
    bl   = _rot(-DRONE_L * 0.35,  DRONE_W * 0.5)
    br   = _rot(-DRONE_L * 0.35, -DRONE_W * 0.5)

    if _drone_patch[0] is not None:
        _drone_patch[0].remove()
    tri = plt.Polygon([nose, bl, br], closed=True,
                      facecolor='#00ffee', edgecolor='white',
                      linewidth=1.5, alpha=0.92, zorder=7)
    ax.add_patch(tri)
    _drone_patch[0] = tri

    # Heading line: from drone centre to 1.5 m ahead
    hx = x + 1.5 * cr
    hy = y + 1.5 * sr
    _heading_line.set_data([x, hx], [y, hy])


# ── Text overlays ─────────────────────────────────────────────────────────────
status_txt = ax.text(
    0.01, 0.99, "waiting for POSE …",
    transform=ax.transAxes, color='#888888',
    fontsize=9, va='top', family='monospace',
    bbox=dict(facecolor='#000000bb', edgecolor='none', pad=5), zorder=10)

visible_txt = ax.text(
    0.99, 0.99, "",
    transform=ax.transAxes, color='#ffdd66',
    fontsize=9, va='top', ha='right', family='monospace',
    bbox=dict(facecolor='#000000bb', edgecolor='none', pad=5), zorder=10)

# Legend
legend_items = [
    mpatches.Patch(facecolor='#ffcc00', label='Marker high (z≥3m)'),
    mpatches.Patch(facecolor='none', edgecolor='#cc8800',
                   linewidth=2, label='Marker low (z<3m)'),
    mpatches.Patch(facecolor='#00ffee', label='Drone (triangle = heading)'),
]
ax.legend(handles=legend_items, loc='lower right',
          facecolor='#111122', edgecolor='#333', labelcolor='white',
          fontsize=8)

plt.tight_layout()
plt.ion()
plt.show()

# ── Main loop ─────────────────────────────────────────────────────────────────
_draw_interval = 1.0 / DRAW_HZ
_last_draw = time.monotonic()

try:
    while plt.fignum_exists(fig.number):
        _poll_serial()

        now = time.monotonic()
        if now - _last_draw < _draw_interval:
            continue
        _last_draw = now

        age = (now - pose_time) if pose_time > 0 else 999.0
        has_fix = pose_time > 0 and age < POSE_TIMEOUT

        if has_fix:
            # Update trail
            if len(trail) >= 2:
                n  = len(trail)
                xs = [p[0] for p in trail]
                ys = [p[1] for p in trail]
                cs = np.linspace(0.0, 1.0, n)
                trail_sc.set_offsets(np.c_[xs, ys])
                trail_sc.set_array(cs)

            _update_drone(pose_x, pose_y, pose_yaw)

            status_txt.set_color('#00ffee')
            status_txt.set_text(
                f"x={pose_x:6.2f} m\n"
                f"y={pose_y:6.2f} m\n"
                f"z={pose_z:6.2f} m\n"
                f"yaw={pose_yaw:+6.1f}°\n"
                f"n={pose_n}"
            )

            if visible:
                vis_lines = ['Visible markers:']
                for mid, dm in visible:
                    vis_lines.append(f"  M{mid}: {float(dm):.2f} m")
                visible_txt.set_text('\n'.join(vis_lines))
            else:
                visible_txt.set_text('')

        else:
            status_txt.set_color('#ff4444')
            status_txt.set_text('no POSE fix')
            visible_txt.set_text('')

        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Done.")
