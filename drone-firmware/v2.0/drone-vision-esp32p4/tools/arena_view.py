#!/usr/bin/env python3
"""
arena_view.py — Top-down arena map viewer for Mach Mind drone swarm.

Shows three overlaid position sources on a live top-down map:
  • CYAN triangle  — P4 ArUco POSE (vision, with yaw heading)
  • YELLOW diamond — what S3 sent to PX4 as vision estimate (sent_px4)
  • ORANGE circle  — PX4 LOCAL_POSITION_NED feedback (px4_ned)

Both serial ports are read simultaneously in the main loop.
Do NOT run stream_view.py at the same time as it shares the P4 port.

Usage:
    python3 arena_view.py [p4_port [s3_port [p4_baud [s3_baud]]]]

Defaults:
    p4_port  = /dev/ttyACM0   (ESP32-P4, CH343 bridge, 921600 baud)
    s3_port  = /dev/ttyACM1   (ESP32-S3, USB-JTAG-Serial, 115200 baud)
    Pass "none" for either port to skip it.

Example — S3 only:
    python3 arena_view.py none /dev/ttyACM0
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

# ── CLI args ──────────────────────────────────────────────────────────────────
_args     = sys.argv[1:]
P4_PORT   = _args[0] if len(_args) > 0 else "/dev/ttyACM0"
S3_PORT   = _args[1] if len(_args) > 1 else "/dev/ttyACM1"
P4_BAUD   = int(_args[2]) if len(_args) > 2 else 921600
S3_BAUD   = int(_args[3]) if len(_args) > 3 else 115200

# ── Constants ─────────────────────────────────────────────────────────────────
TRAIL_LEN    = 200
POSE_TIMEOUT = 2.0   # seconds before marking a source stale
DRAW_HZ      = 20

ARENA_W = 20.0
ARENA_H = 10.0

# ArUco marker world map — from aruco_pose.cpp
# (id, x_m, y_m, z_m)
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

# ── Serial connections ────────────────────────────────────────────────────────
def _open_port(port, baud, label):
    import os
    if port.lower() == 'none':
        print(f"[{label}] skipped (none)")
        return None
    if not os.path.exists(port):
        print(f"[{label}] skipped — {port} not found")
        return None
    try:
        s = serial.Serial(port, baud, timeout=0.05)
        print(f"[{label}] opened {port} @ {baud}")
        atexit.register(s.close)
        return s
    except Exception as e:
        print(f"[{label}] could not open {port}: {e}")
        return None

ser_p4 = _open_port(P4_PORT, P4_BAUD, "P4")
ser_s3 = _open_port(S3_PORT, S3_BAUD, "S3")

signal.signal(signal.SIGTERM, lambda _s, _f: sys.exit(0))
signal.signal(signal.SIGHUP,  lambda _s, _f: sys.exit(0))

# ── State ─────────────────────────────────────────────────────────────────────
# P4 ArUco POSE
p4_x = p4_y = p4_z = p4_yaw = 0.0
p4_n = 0
p4_time = 0.0
p4_visible = []
p4_trail = deque(maxlen=TRAIL_LEN)

# S3 [POS] line — three positions
s3_aruco_x = s3_aruco_y = 0.0    # P4_aruco as received by S3 (arena coords)
s3_sent_x  = s3_sent_y  = 0.0    # what S3 sent to PX4 (NED-relative coords)
s3_sent_yaw = 0.0                 # yaw sent to PX4 (vision_yaw, degrees)
s3_ned_x   = s3_ned_y   = 0.0    # PX4 LOCAL_POSITION_NED feedback (NED-relative)
s3_ned_yaw  = 0.0                 # PX4's own yaw from ATTITUDE msg (degrees)
s3_offset_x = s3_offset_y = 0.0  # NED origin in arena coords (= arming position)
s3_time = 0.0

# ── Parsers ───────────────────────────────────────────────────────────────────
_POSE_RE   = re.compile(
    r'POSE:(\d+):([-\d.]+):([-\d.]+):([-\d.]+)'
    r':([-\d.]+):([-\d.]+):([-\d.]+):([-\d.]+)'
)
_MARKER_RE = re.compile(r'M(\d+):([\d.]+)m')

# [POS] P4_aruco=(x,y)  sent_px4=(x,y,yaw=d)  px4_ned=(x,y,yaw=d)  offset=(x,y)
_POS_RE = re.compile(
    r'\[POS\].*P4_aruco=\(([-\d.]+),([-\d.]+)\)'
    r'.*sent_px4=\(([-\d.]+),([-\d.]+),yaw=([-\d.]+)\)'
    r'.*px4_ned=\(([-\d.]+),([-\d.]+),yaw=([-\d.]+)\)'
    r'.*offset=\(([-\d.]+),([-\d.]+)\)'
)


def _quat_to_yaw(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny, cosy))


def _parse_p4(line):
    global p4_x, p4_y, p4_z, p4_yaw, p4_n, p4_time, p4_visible
    m = _POSE_RE.search(line)
    if not m:
        return
    p4_n   = int(m.group(1))
    p4_x   = float(m.group(2))
    p4_y   = float(m.group(3))
    p4_z   = float(m.group(4))
    qx, qy, qz, qw = (float(m.group(i)) for i in range(5, 9))
    p4_yaw  = _quat_to_yaw(qx, qy, qz, qw)
    p4_time = time.monotonic()
    p4_visible = _MARKER_RE.findall(line)
    p4_trail.append((p4_x, p4_y))


def _parse_s3(line):
    global s3_aruco_x, s3_aruco_y, s3_sent_x, s3_sent_y, s3_sent_yaw
    global s3_ned_x, s3_ned_y, s3_ned_yaw, s3_offset_x, s3_offset_y, s3_time
    m = _POS_RE.search(line)
    if not m:
        return
    s3_aruco_x  = float(m.group(1));  s3_aruco_y  = float(m.group(2))
    s3_sent_x   = float(m.group(3));  s3_sent_y   = float(m.group(4))
    s3_sent_yaw = float(m.group(5))
    s3_ned_x    = float(m.group(6));  s3_ned_y    = float(m.group(7))
    s3_ned_yaw  = float(m.group(8))
    s3_offset_x = float(m.group(9));  s3_offset_y = float(m.group(10))
    s3_time = time.monotonic()


# Non-blocking byte-level reader.
#
# binary_safe=True  (P4): flush on LF only; clear buffer on any non-printable
#                         byte so binary camera frames don't corrupt the buffer.
# binary_safe=False (S3): flush on LF and CR (\r-prefixed printf lines);
#                         skip non-printable bytes so ANSI color codes from
#                         ESP_LOGI don't wipe the buffer before the final \n.
def _make_reader(parse_fn, binary_safe=True):
    buf = bytearray()
    def _flush():
        if buf:
            try:
                parse_fn(buf.decode('ascii', errors='replace'))
            except Exception:
                pass
        buf.clear()
    def poll(ser):
        nonlocal buf
        try:
            waiting = ser.in_waiting
            data = ser.read(waiting if waiting > 0 else 1)
        except Exception:
            return
        for c in data:
            if c == 0x0A:                  # LF — flush always
                _flush()
            elif c == 0x0D:                # CR
                if binary_safe:
                    pass                   # P4: ignore CR
                else:
                    _flush()               # S3: \r ends a printf line
            elif 0x20 <= c < 0x80:        # printable ASCII
                buf.append(c)
                if len(buf) > 1024:
                    buf.clear()
            else:                          # non-printable (ANSI ESC, etc.)
                if binary_safe:
                    buf.clear()            # P4: binary frame byte → reset
                # S3: silently skip (ANSI color codes must not clear buffer)
    return poll

_poll_p4 = _make_reader(_parse_p4, binary_safe=True)  if ser_p4 else None
_poll_s3 = _make_reader(_parse_s3, binary_safe=False) if ser_s3 else None

# ── Matplotlib figure ─────────────────────────────────────────────────────────
PAD = 1.5
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

for x_pos in [0, 5, 10, 15, 20]:
    ax.axvline(x=x_pos, color='#1a2a4a', linewidth=0.5, zorder=1)
    ax.text(x_pos, -0.6, str(x_pos), color='#555', ha='center',
            fontsize=8, family='monospace')
for y_pos in [0, 5, 10]:
    ax.axhline(y=y_pos, color='#1a2a4a', linewidth=0.5, zorder=1)
    ax.text(-0.8, y_pos, str(y_pos), color='#555', va='center',
            fontsize=8, family='monospace')

# ── ArUco markers ─────────────────────────────────────────────────────────────
_xy_groups: dict = {}
for mid, mx, my, mz in MARKERS:
    _xy_groups.setdefault((mx, my), []).append((mid, mz))

for (mx, my), entries in _xy_groups.items():
    entries.sort(key=lambda e: -e[1])
    label = '/'.join(str(e[0]) for e in entries)
    for mid, mz in entries:
        color = '#ffcc00' if mz >= 3.0 else '#cc8800'
        fill  = color     if mz >= 3.0 else 'none'
        ax.plot(mx, my, 's', markersize=11, color=color,
                markerfacecolor=fill, markeredgewidth=2, zorder=3)
    ax.text(mx, my + 0.55, f'M{label}', color='#ffdd66',
            fontsize=7.5, ha='center', va='bottom',
            family='monospace', zorder=4,
            bbox=dict(facecolor='#00000099', edgecolor='none', pad=1))

# ── P4 trail scatter ──────────────────────────────────────────────────────────
trail_sc = ax.scatter([], [], s=12, c=[], cmap='cool',
                      vmin=0.0, vmax=1.0, alpha=0.55, zorder=5)

# ── P4 drone triangle (cyan) ──────────────────────────────────────────────────
_drone_patch = [None]
_heading_line, = ax.plot([], [], '-', color='#00ffee', linewidth=2.0,
                         alpha=0.8, zorder=8)
DRONE_L = 0.9
DRONE_W = 0.5


def _update_drone(x, y, yaw_deg):
    r = math.radians(yaw_deg + 90.0)   # _quat_to_yaw = camera-right angle; +90° converts to camera-forward
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
    hx = x + 1.5 * cr
    hy = y + 1.5 * sr
    _heading_line.set_data([x, hx], [y, hy])


# ── NED origin cross (arming position in arena coords) ───────────────────
ned_origin_plot, = ax.plot([], [], '+', markersize=18, color='#ff8800',
                           markeredgewidth=2.5, zorder=8)
ned_origin_label = ax.text(0, 0, '', color='#ff8800', fontsize=7.5,
                           ha='center', va='bottom', family='monospace', zorder=10,
                           bbox=dict(facecolor='#00000099', edgecolor='none', pad=1))

# ── S3 sent_px4 marker (yellow diamond) ──────────────────────────────────────
sent_dot, = ax.plot([], [], 'D', markersize=12, color='#ffee00',
                    markeredgecolor='white', markeredgewidth=1.2,
                    zorder=9, label='S3→PX4 sent')
sent_heading, = ax.plot([], [], '-', color='#ffee00', linewidth=2.0,
                        alpha=0.85, zorder=9)
sent_label = ax.text(0, 0, '', color='#ffee00', fontsize=7.5,
                     ha='center', va='top', family='monospace', zorder=10,
                     bbox=dict(facecolor='#00000099', edgecolor='none', pad=1))

# ── S3 px4_ned marker (orange circle) ────────────────────────────────────────
ned_dot, = ax.plot([], [], 'o', markersize=14, color='#ff8800',
                   markeredgecolor='white', markeredgewidth=1.2,
                   markerfacecolor='none', zorder=9, label='PX4 NED')
ned_heading, = ax.plot([], [], '-', color='#ff8800', linewidth=2.0,
                       alpha=0.85, zorder=9)
ned_label = ax.text(0, 0, '', color='#ff8800', fontsize=7.5,
                    ha='center', va='top', family='monospace', zorder=10,
                    bbox=dict(facecolor='#00000099', edgecolor='none', pad=1))

# ── Text overlays ─────────────────────────────────────────────────────────────
p4_txt = ax.text(
    0.01, 0.99, "P4: waiting…",
    transform=ax.transAxes, color='#00ffee',
    fontsize=9, va='top', family='monospace',
    bbox=dict(facecolor='#000000bb', edgecolor='none', pad=5), zorder=10)

s3_txt = ax.text(
    0.01, 0.70, "S3: waiting…",
    transform=ax.transAxes, color='#ffee00',
    fontsize=9, va='top', family='monospace',
    bbox=dict(facecolor='#000000bb', edgecolor='none', pad=5), zorder=10)

visible_txt = ax.text(
    0.99, 0.99, "",
    transform=ax.transAxes, color='#ffdd66',
    fontsize=9, va='top', ha='right', family='monospace',
    bbox=dict(facecolor='#000000bb', edgecolor='none', pad=5), zorder=10)

# ── Legend ────────────────────────────────────────────────────────────────────
legend_items = [
    mpatches.Patch(facecolor='#ffcc00', label='ArUco marker (z≥3m)'),
    mpatches.Patch(facecolor='none', edgecolor='#cc8800',
                   linewidth=2, label='ArUco marker (z<3m)'),
    mpatches.Patch(facecolor='#00ffee', label='P4 vision (triangle=heading)'),
    mpatches.Patch(facecolor='#ffee00', label='S3 → PX4 sent (arena coords)'),
    mpatches.Patch(facecolor='none', edgecolor='#ff8800',
                   linewidth=2, label='PX4 LOCAL_NED (arena coords)'),
    mpatches.Patch(facecolor='none', edgecolor='#ff8800',
                   linewidth=0, label='+ = NED origin (arming pos)'),
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
        if _poll_p4 and ser_p4:
            _poll_p4(ser_p4)
        if _poll_s3 and ser_s3:
            _poll_s3(ser_s3)

        now = time.monotonic()
        if now - _last_draw < _draw_interval:
            continue
        _last_draw = now

        # ── P4 update ─────────────────────────────────────────────────────────
        p4_age   = (now - p4_time) if p4_time > 0 else 999.0
        p4_fix   = p4_time > 0 and p4_age < POSE_TIMEOUT

        if p4_fix:
            if len(p4_trail) >= 2:
                xs = [p[0] for p in p4_trail]
                ys = [p[1] for p in p4_trail]
                cs = np.linspace(0.0, 1.0, len(p4_trail))
                trail_sc.set_offsets(np.c_[xs, ys])
                trail_sc.set_array(cs)
            _update_drone(p4_x, p4_y, p4_yaw)
            p4_txt.set_color('#00ffee')
            p4_txt.set_text(
                f"P4 vision\n"
                f"  x={p4_x:.2f}  y={p4_y:.2f}  z={p4_z:.2f} m\n"
                f"  yaw={p4_yaw:+.1f}°  n={p4_n}"
            )
            if p4_visible:
                vis_lines = ['Visible:']
                for mid, dm in p4_visible:
                    vis_lines.append(f"  M{mid}: {float(dm):.2f}m")
                visible_txt.set_text('\n'.join(vis_lines))
            else:
                visible_txt.set_text('')
        else:
            p4_txt.set_color('#ff4444')
            p4_txt.set_text('P4: no fix' if ser_p4 else 'P4: not connected')
            visible_txt.set_text('')

        # ── S3 update ─────────────────────────────────────────────────────────
        s3_age = (now - s3_time) if s3_time > 0 else 999.0
        s3_fix = s3_time > 0 and s3_age < 5.0   # [POS] logs every 2 s

        s3_localized = s3_fix and (s3_aruco_x != 0.0 or s3_aruco_y != 0.0)

        def _s3_heading(x, y, yaw_deg, length=1.2):
            # S3 vision_yaw = atan2(camera_fwd_y, camera_fwd_x) — already forward;
            # no +90° correction needed (unlike P4 quat-yaw which is camera-right).
            r = math.radians(yaw_deg)
            return [x, x + length * math.cos(r)], [y, y + length * math.sin(r)]

        if s3_localized:
            # Convert NED-relative coords to arena coords by adding NED origin offset
            sent_ax = s3_sent_x + s3_offset_x
            sent_ay = s3_sent_y + s3_offset_y
            ned_ax  = s3_ned_x  + s3_offset_x
            ned_ay  = s3_ned_y  + s3_offset_y

            # sent_px4 — yellow diamond + heading (arena coords)
            sent_dot.set_data([sent_ax], [sent_ay])
            hx, hy = _s3_heading(sent_ax, sent_ay, s3_sent_yaw)
            sent_heading.set_data(hx, hy)
            sent_label.set_position((sent_ax, sent_ay - 0.6))
            sent_label.set_text(f"sent {s3_sent_yaw:.0f}°\n({sent_ax:.2f},{sent_ay:.2f})")

            # px4_ned — orange circle + heading (arena coords)
            ned_dot.set_data([ned_ax], [ned_ay])
            hx, hy = _s3_heading(ned_ax, ned_ay, s3_ned_yaw)
            ned_heading.set_data(hx, hy)
            ned_label.set_position((ned_ax, ned_ay - 0.6))
            ned_label.set_text(f"NED {s3_ned_yaw:.0f}°\n({ned_ax:.2f},{ned_ay:.2f})")

            # NED origin cross at arming position
            ned_origin_plot.set_data([s3_offset_x], [s3_offset_y])
            ned_origin_label.set_position((s3_offset_x, s3_offset_y + 0.5))
            ned_origin_label.set_text(f"NED(0,0)\n({s3_offset_x:.1f},{s3_offset_y:.1f})")

            s3_txt.set_color('#ffee00')
            s3_txt.set_text(
                f"S3 [POS]  (age {s3_age:.0f}s)\n"
                f"  NED origin / arming  ({s3_offset_x:.1f}, {s3_offset_y:.1f})\n"
                f"  P4→S3  aruco        ({s3_aruco_x:.2f}, {s3_aruco_y:.2f})\n"
                f"  S3→PX4 sent (arena) ({sent_ax:.2f}, {sent_ay:.2f})  yaw={s3_sent_yaw:.1f}°\n"
                f"  PX4    NED  (arena) ({ned_ax:.2f}, {ned_ay:.2f})  yaw={s3_ned_yaw:.1f}°"
            )
        else:
            sent_dot.set_data([], [])
            sent_heading.set_data([], [])
            sent_label.set_text('')
            ned_dot.set_data([], [])
            ned_heading.set_data([], [])
            ned_label.set_text('')
            ned_origin_plot.set_data([], [])
            ned_origin_label.set_text('')
            s3_txt.set_color('#888888')
            s3_txt.set_text('S3: no [POS] data' if ser_s3 else 'S3: not connected')

        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:
    pass
finally:
    if ser_p4:
        ser_p4.close()
    if ser_s3:
        ser_s3.close()
    print("Done.")
