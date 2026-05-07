"""
PFC CAN Scope PRO v5  +  PI GAINS TX  +  VOLT GAINS TX  +  VDC REF TX
=======================================================================
"""

import can
import struct
import threading
import collections
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, TextBox
from matplotlib.gridspec import GridSpec

# ═══════════════════════════════════════════════
# KONFIGURACJA
# ═══════════════════════════════════════════════

CHANNEL       = "PCAN_USBBUS1"
BITRATE       = 500_000
BUFFER_SIZE   = 4000

CAN_ID_321    = 0x321
CAN_ID_322    = 0x322
CAN_ID_323    = 0x323
CAN_ID_GAINS  = 0x400
CAN_ID_VGAINS = 0x401
CAN_ID_VDCREF = 0x402

# ═══════════════════════════════════════════════
# BUFORY
# ═══════════════════════════════════════════════

KEYS = ["iL_A", "i_ref", "d_pi", "d_ff", "i_ref_amp"]

buf       = {k: collections.deque(maxlen=BUFFER_SIZE) for k in KEYS}
time_buf  = collections.deque(maxlen=BUFFER_SIZE)
_buf_lock = threading.Lock()

# ═══════════════════════════════════════════════
# CAN
# ═══════════════════════════════════════════════

running_flag = threading.Event()
running_flag.set()

_rx_bus   = None
_bus_lock = threading.Lock()

def can_reader():
    global _rx_bus
    try:
        with _bus_lock:
            _rx_bus = can.interface.Bus(interface="pcan", channel=CHANNEL, bitrate=BITRATE)
    except Exception as e:
        print(f"[CAN] Błąd otwarcia bus: {e}")
        return

    print("[CAN] Bus otwarty.")
    latest = {}

    while True:
        if not running_flag.is_set():
            time.sleep(0.02)
            latest.clear()
            continue

        try:
            with _bus_lock:
                msg = _rx_bus.recv(timeout=0.05)
        except Exception as e:
            print(f"[CAN] recv error: {e}")
            time.sleep(0.1)
            continue

        if msg is None:
            continue

        aid = msg.arbitration_id
        d   = bytes(msg.data)

        try:
            if   aid == CAN_ID_321 and len(d) >= 8:
                latest[0x321] = struct.unpack_from("ff", d, 0)
            elif aid == CAN_ID_322 and len(d) >= 8:
                latest[0x322] = struct.unpack_from("ff", d, 0)
            elif aid == CAN_ID_323 and len(d) >= 4:
                latest[0x323] = struct.unpack_from("f",  d, 0)
        except struct.error:
            continue

        if 0x321 in latest and 0x322 in latest and 0x323 in latest:
            d_pi, d_ff  = latest[0x321]
            i_ref, iL_A = latest[0x322]
            vm,         = latest[0x323]
            ts          = time.perf_counter()

            with _buf_lock:
                buf["d_pi"].append(d_pi)
                buf["d_ff"].append(d_ff)
                buf["i_ref"].append(i_ref)
                buf["iL_A"].append(iL_A)
                buf["i_ref_amp"].append(vm)
                time_buf.append(ts)

            latest.clear()


def _can_send(can_id, kp, ki, label):
    try:
        data = struct.pack("<ff", kp, ki)
        msg  = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        with _bus_lock:
            if _rx_bus is not None:
                _rx_bus.send(msg, timeout=0.5)
                print(f"[CAN TX] 0x{can_id:03X} [{label}] → Kp={kp:.5f}  Ki={ki:.5f}")
            else:
                raise RuntimeError("Bus nie jest otwarty – kliknij START")
    except Exception as e:
        print(f"[CAN TX] Błąd ({label}): {e}")
        raise


def can_send_gains(kp, ki):
    _can_send(CAN_ID_GAINS, kp, ki, "I-ctrl")


def can_send_volt_gains(kp, ki):
    _can_send(CAN_ID_VGAINS, kp, ki, "V-ctrl")


def can_send_vdc_ref(vref):
    try:
        data = struct.pack("<f", vref) + b'\x00' * 4
        msg  = can.Message(arbitration_id=CAN_ID_VDCREF, data=data, is_extended_id=False)
        with _bus_lock:
            if _rx_bus is not None:
                _rx_bus.send(msg, timeout=0.5)
                print(f"[CAN TX] 0x{CAN_ID_VDCREF:03X} → Vdc_ref={vref:.1f}V")
            else:
                raise RuntimeError("Bus nie jest otwarty – kliknij START")
    except Exception as e:
        print(f"[CAN TX] Błąd (Vdc_ref): {e}")
        raise

# ═══════════════════════════════════════════════
# DEFINICJE KANAŁÓW
# ═══════════════════════════════════════════════

CH_DEF = [
    ("iL_A",      "iL_A  – Inductor Current [A]",       "#00f5d4",  0.0,  40.0),
    ("i_ref",     "i_ref – Current Reference [A]",      "#ffd166",  0.0,  40.0),
    ("d_pi",      "d_pi  – Duty PI output",             "#f15bb5",  0.0,   0.4),
    ("d_ff",      "d_ff  – Vdc ref [V]",                "#06d6a0",  0.0, 200.0),
    ("i_ref_amp", "i_ref_amp – Voltage PI output [A]",  "#a0c4ff",  0.0,  15.0),
]
N_CH = len(CH_DEF)

# ═══════════════════════════════════════════════
# MATPLOTLIB STYL
# ═══════════════════════════════════════════════

plt.rcParams.update({
    "figure.facecolor": "#080c12",
    "axes.facecolor":   "#0b1120",
    "axes.edgecolor":   "#1c2a3c",
    "axes.labelcolor":  "#667788",
    "xtick.color":      "#445566",
    "ytick.color":      "#445566",
    "grid.color":       "#141f30",
    "font.size":        9,
    "font.family":      "monospace",
})

fig = plt.figure(figsize=(15, 9))
fig.canvas.manager.set_window_title("PFC CAN Scope PRO v5 + PI & Volt Gains TX + Vdc Ref")

gs = GridSpec(N_CH, 1, figure=fig,
              top=0.98, bottom=0.42, left=0.07, right=0.98, hspace=0.60)

axes       = []
lines      = []
cur_lines  = []
cur_dots   = []
cur_labels = []
t_rms_list = []
t_inf_list = []

for i, (key, title, color, yc, ys) in enumerate(CH_DEF):
    ax = fig.add_subplot(gs[i])
    ln, = ax.plot([], [], lw=1.2, color=color, antialiased=True)
    ax.set_title(title, color=color, fontsize=9, pad=2,
                 loc="left", fontweight="bold")
    ax.grid(True, alpha=0.30)
    axes.append(ax)
    lines.append(ln)

    cl = ax.axvline(x=0, color="#ffee00", lw=1.0, ls="--", visible=False, zorder=5)
    cur_lines.append(cl)

    cd, = ax.plot([], [], "o", color="#ffee00", ms=5, zorder=6, visible=False)
    cur_dots.append(cd)

    clbl = ax.text(0.01, 0.60, "", transform=ax.transAxes,
                   color="#ffee00", fontsize=8, fontfamily="monospace",
                   bbox=dict(boxstyle="round,pad=0.2", fc="#080c12", ec="#ffee00", alpha=0.8),
                   visible=False, zorder=7)
    cur_labels.append(clbl)

    t_rms = ax.text(0.01, 0.88, "", transform=ax.transAxes,
                    color=color, fontsize=8, weight="bold", fontfamily="monospace")
    t_inf = ax.text(0.60, 0.88, "", transform=ax.transAxes,
                    color="#445566", fontsize=8, fontfamily="monospace")
    t_rms_list.append(t_rms)
    t_inf_list.append(t_inf)

# ═══════════════════════════════════════════════
# OVERLAY STATE
# ═══════════════════════════════════════════════

overlay_map   = {i: set() for i in range(N_CH)}
overlay_lines = {i: {}    for i in range(N_CH)}

for i in range(N_CH):
    for j in range(N_CH):
        if j == i:
            overlay_lines[i][j] = None
            continue
        color_j = CH_DEF[j][2]
        ol, = axes[i].plot([], [], lw=1.0, color=color_j,
                           ls="--", alpha=0.75, visible=False,
                           label=CH_DEF[j][0], zorder=3)
        overlay_lines[i][j] = ol

# ═══════════════════════════════════════════════
# SHARED VIEW STATE
# ═══════════════════════════════════════════════

shared_x = {"center": None, "span": 500, "auto": True}

ch_y = []
for (key, title, color, yc, ys) in CH_DEF:
    ch_y.append({"center": yc, "span": ys, "auto": True})

cursor_x = {"pos": None, "active": False}

# ═══════════════════════════════════════════════
# VIEW HELPERS
# ═══════════════════════════════════════════════

def apply_views(n):
    sx = shared_x
    if sx["center"] is None:
        sx["center"] = max(n - 1, 0)
    xc, xs = sx["center"], sx["span"]
    x0 = max(0.0, xc - xs / 2)
    x1 = x0 + xs
    if x1 > n:
        x1 = float(n)
        x0 = max(0.0, x1 - xs)
    for i, ax in enumerate(axes):
        ax.set_xlim(x0, x1)
        cy, sy = ch_y[i]["center"], ch_y[i]["span"]
        ax.set_ylim(cy - sy / 2, cy + sy / 2)

# ═══════════════════════════════════════════════
# INTERAKCJA
# ═══════════════════════════════════════════════

def get_mods(event):
    key_str = (getattr(event, "key", "") or "").lower()
    shift = "shift" in key_str
    ctrl  = "control" in key_str or "ctrl" in key_str
    if not shift and not ctrl:
        try:
            from PyQt6.QtCore import Qt
            mods = event.guiEvent.modifiers() if event.guiEvent else None
            if mods is not None:
                shift = bool(mods & Qt.KeyboardModifier.ShiftModifier)
                ctrl  = bool(mods & Qt.KeyboardModifier.ControlModifier)
        except Exception:
            pass
    if not shift and not ctrl:
        try:
            from PyQt5.QtCore import Qt
            mods = event.guiEvent.modifiers() if event.guiEvent else None
            if mods is not None:
                shift = bool(mods & Qt.ShiftModifier)
                ctrl  = bool(mods & Qt.ControlModifier)
        except Exception:
            pass
    return shift, ctrl


def on_scroll(event):
    if event.inaxes is None:
        return
    ch_idx = next((i for i, ax in enumerate(axes) if event.inaxes is ax), None)
    if ch_idx is None:
        return
    factor = 1.25 if event.button == "down" else (1 / 1.25)
    shift, ctrl = get_mods(event)
    if shift:
        v = ch_y[ch_idx]
        v["auto"] = False
        if event.ydata is not None:
            v["center"] = event.ydata * (1 - 1/factor) + v["center"] * (1/factor)
        v["span"] = max(1e-9, v["span"] * factor)
    else:
        sx = shared_x
        sx["auto"] = False
        for v in ch_y:
            v["auto"] = False
        if ctrl:
            step = sx["span"] * 0.12 * (1 if event.button == "down" else -1)
            sx["center"] = (sx["center"] or 0) + step
        else:
            if event.xdata is not None:
                sx["center"] = event.xdata * (1 - 1/factor) + (sx["center"] or event.xdata) * (1/factor)
            sx["span"] = max(10, sx["span"] * factor)
    fig.canvas.draw_idle()


_drag     = {"ch": None, "y0": None, "yc0": None, "x0": None, "xc0": None}
_cur_drag = {"active": False}

def on_press(event):
    if event.inaxes is None:
        return
    ch_idx = next((i for i, ax in enumerate(axes) if event.inaxes is ax), None)
    if ch_idx is None:
        return
    if event.button == 1:
        _drag["ch"]  = ch_idx
        _drag["y0"]  = event.ydata
        _drag["yc0"] = ch_y[ch_idx]["center"]
        _drag["x0"]  = event.xdata
        _drag["xc0"] = shared_x["center"]
        ch_y[ch_idx]["auto"] = False
        shared_x["auto"] = False
        for v in ch_y:
            v["auto"] = False
    elif event.button == 3:
        if event.xdata is not None:
            cursor_x["pos"]     = event.xdata
            cursor_x["active"]  = True
            _cur_drag["active"] = True
            fig.canvas.draw_idle()

def on_motion(event):
    if _cur_drag["active"]:
        if event.xdata is not None:
            cursor_x["pos"] = event.xdata
            fig.canvas.draw_idle()
        return
    if _drag["ch"] is None or event.inaxes is None:
        return
    ch_idx = _drag["ch"]
    if event.inaxes is not axes[ch_idx]:
        return
    if event.xdata is None or event.ydata is None:
        return
    dx = event.xdata - _drag["x0"]
    dy = event.ydata - _drag["y0"]
    ch_y[ch_idx]["center"] = _drag["yc0"] - dy
    shared_x["center"]     = _drag["xc0"] - dx
    fig.canvas.draw_idle()

def on_release(event):
    if event.button == 1:
        _drag["ch"] = None
    elif event.button == 3:
        _cur_drag["active"] = False

def on_key(event):
    if event.key == "escape":
        cursor_x["active"] = False
        for cl in cur_lines:
            cl.set_visible(False)
        for cd in cur_dots:
            cd.set_visible(False)
        for clbl in cur_labels:
            clbl.set_visible(False)
        fig.canvas.draw_idle()

fig.canvas.mpl_connect("scroll_event",         on_scroll)
fig.canvas.mpl_connect("button_press_event",   on_press)
fig.canvas.mpl_connect("motion_notify_event",  on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)
fig.canvas.mpl_connect("key_press_event",      on_key)

# ═══════════════════════════════════════════════
# ANIMATION UPDATE
# ═══════════════════════════════════════════════

def update(_frame):
    with _buf_lock:
        n = len(buf["iL_A"])
        if n < 2:
            return []
        snap = {k: np.array(buf[k]) for k in KEYS}

    x = np.arange(n)

    if shared_x["auto"]:
        shared_x["center"] = float(n - 1)

    for i, (key, title, color, yc0, ys0) in enumerate(CH_DEF):
        data = snap[key]
        lines[i].set_data(x, data)
        v = ch_y[i]
        if v["auto"]:
            lo, hi  = float(data.min()), float(data.max())
            center  = (hi + lo) / 2.0
            span    = max(hi - lo, abs(center) * 0.05, 1e-6)
            v["center"] = center
            v["span"]   = span * 2.8
        rms = float(np.sqrt(np.mean(data**2)))
        mn  = float(data.min())
        mx  = float(data.max())
        t_rms_list[i].set_text(f"RMS={rms:.3f}  min={mn:.3f}  max={mx:.3f}")
        t_inf_list[i].set_text(
            f"W={int(shared_x['span'])} smp  V/div={ch_y[i]['span']/10:.4f}"
        )

    apply_views(n)

    if cursor_x["active"] and cursor_x["pos"] is not None:
        cx = cursor_x["pos"]
        xi = int(round(cx))
        for i, (key, title, color, yc0, ys0) in enumerate(CH_DEF):
            cur_lines[i].set_xdata([cx, cx])
            cur_lines[i].set_visible(True)
            if 0 <= xi < n:
                val = snap[key][xi]
                cur_dots[i].set_data([xi], [val])
                cur_dots[i].set_visible(True)
                cur_labels[i].set_text(f"x={xi}\n{key}={val:.4f}")
                cur_labels[i].set_visible(True)
            else:
                cur_dots[i].set_visible(False)
                cur_labels[i].set_visible(False)

    all_overlay = []
    for i in range(N_CH):
        for j in overlay_map[i]:
            ol = overlay_lines[i][j]
            if ol is None:
                continue
            ol.set_data(x, snap[CH_DEF[j][0]])
            ol.set_visible(True)
            all_overlay.append(ol)
        for j in range(N_CH):
            if j not in overlay_map[i] and overlay_lines[i].get(j) is not None:
                overlay_lines[i][j].set_visible(False)
                all_overlay.append(overlay_lines[i][j])

    return (lines + t_rms_list + t_inf_list +
            cur_lines + cur_dots + cur_labels + all_overlay)


ani = animation.FuncAnimation(fig, update, interval=60, blit=False, cache_frame_data=False)

# ═══════════════════════════════════════════════
# GUI – STAŁE / HELPERY
# ═══════════════════════════════════════════════

C_BTN     = "#0b1520"
C_START   = "#0e6e50"
C_STOP    = "#7a1525"
C_BLUE    = "#163d72"
C_GRAY    = "#223040"
C_PURP    = "#3a1a5a"
C_OVL_ON  = "#1a3a20"
C_OVL_OFF = "#0b1520"

def make_btn(rect, label, hcol, fsize=9, color="#b0c8e0"):
    ax_b = plt.axes(rect)
    b = Button(ax_b, label, color=C_BTN, hovercolor=hcol)
    b.label.set_color(color)
    b.label.set_fontsize(fsize)
    b.label.set_weight("bold")
    b.label.set_fontfamily("monospace")
    return b

# ═══════════════════════════════════════════════
# LAYOUT
#
#  ROW_GAINS   y=0.405  │ CURRENT PI GAINS  → 0x400         │
#  ROW_VGAINS  y=0.360  │ VOLTAGE PI GAINS  → 0x401         │
#  ROW_VDCREF  y=0.315  │ VDC REF           → 0x402         │
#  ROW_A       y=0.270  │ START │ STOP │ AUTO ALL │ CLR │ SAVE │
#  ROW_B       y=0.220  │ AUTO per kanał                     │
#  ROW_C       y=0.170  │ Ctr / Spn / SET per kanał          │
#  ROW_D       y=0.010  │ OVERLAY macierz                    │
# ═══════════════════════════════════════════════

BTN_H      = 0.038
ROW_GAINS  = 0.405
ROW_VGAINS = 0.360
ROW_VDCREF = 0.315
ROW_A      = 0.270
ROW_B      = 0.220
ROW_C      = 0.170
ROW_D      = 0.010

# ── ROW_GAINS: PI regulatora PRĄDU ──────────────────────────────────

fig.text(0.01, ROW_GAINS + BTN_H + 0.003,
         "── CURRENT PI GAINS  →  CAN 0x400 ──",
         color="#667788", fontsize=8, fontfamily="monospace", va="bottom")

ax_kp = plt.axes([0.01, ROW_GAINS, 0.10, BTN_H])
tb_kp = TextBox(ax_kp, "Kp:", initial="0.0700")
tb_kp.label.set_color("#ffd166")
tb_kp.label.set_fontsize(8)
tb_kp.text_disp.set_color("#ccddee")
tb_kp.text_disp.set_fontfamily("monospace")
tb_kp.text_disp.set_fontsize(9)

ax_ki = plt.axes([0.13, ROW_GAINS, 0.10, BTN_H])
tb_ki = TextBox(ax_ki, "Ki:", initial="1.0000")
tb_ki.label.set_color("#06d6a0")
tb_ki.label.set_fontsize(8)
tb_ki.text_disp.set_color("#ccddee")
tb_ki.text_disp.set_fontfamily("monospace")
tb_ki.text_disp.set_fontsize(9)

btn_gains = make_btn([0.25, ROW_GAINS, 0.08, BTN_H],
                     "📤 SEND", "#1a5a2a", fsize=9, color="#00f5a0")

gains_status = fig.text(
    0.34, ROW_GAINS + BTN_H * 0.45,
    "–  wpisz Kp / Ki i kliknij SEND",
    color="#445566", fontsize=8, fontfamily="monospace", va="center"
)

# ── ROW_VGAINS: PI regulatora NAPIĘCIA ──────────────────────────────

fig.text(0.01, ROW_VGAINS + BTN_H + 0.003,
         "── VOLTAGE PI GAINS  →  CAN 0x401 ──",
         color="#667788", fontsize=8, fontfamily="monospace", va="bottom")

ax_kp_v = plt.axes([0.01, ROW_VGAINS, 0.10, BTN_H])
tb_kp_v = TextBox(ax_kp_v, "Kp:", initial="0.0500")
tb_kp_v.label.set_color("#a0c4ff")
tb_kp_v.label.set_fontsize(8)
tb_kp_v.text_disp.set_color("#ccddee")
tb_kp_v.text_disp.set_fontfamily("monospace")
tb_kp_v.text_disp.set_fontsize(9)

ax_ki_v = plt.axes([0.13, ROW_VGAINS, 0.10, BTN_H])
tb_ki_v = TextBox(ax_ki_v, "Ki:", initial="0.5000")
tb_ki_v.label.set_color("#f15bb5")
tb_ki_v.label.set_fontsize(8)
tb_ki_v.text_disp.set_color("#ccddee")
tb_ki_v.text_disp.set_fontfamily("monospace")
tb_ki_v.text_disp.set_fontsize(9)

btn_vgains = make_btn([0.25, ROW_VGAINS, 0.08, BTN_H],
                      "📤 SEND", "#1a2a5a", fsize=9, color="#a0c4ff")

volt_gains_status = fig.text(
    0.34, ROW_VGAINS + BTN_H * 0.45,
    "–  wpisz Kp_v / Ki_v i kliknij SEND",
    color="#445566", fontsize=8, fontfamily="monospace", va="center"
)

# ── ROW_VDCREF: Vdc_ref ─────────────────────────────────────────────

fig.text(0.01, ROW_VDCREF + BTN_H + 0.003,
         "── VDC REF  →  CAN 0x402 ──",
         color="#667788", fontsize=8, fontfamily="monospace", va="bottom")

ax_vdc = plt.axes([0.01, ROW_VDCREF, 0.10, BTN_H])
tb_vdc = TextBox(ax_vdc, "Vdc:", initial="")
tb_vdc.label.set_color("#ffd166")
tb_vdc.label.set_fontsize(8)
tb_vdc.text_disp.set_color("#ccddee")
tb_vdc.text_disp.set_fontfamily("monospace")
tb_vdc.text_disp.set_fontsize(9)

btn_vdc = make_btn([0.13, ROW_VDCREF, 0.08, BTN_H],
                   "📤 SEND", "#5a3a1a", fsize=9, color="#ffd166")

vdc_status = fig.text(
    0.22, ROW_VDCREF + BTN_H * 0.45,
    "–  wpisz Vdc_ref [V] i kliknij SEND",
    color="#445566", fontsize=8, fontfamily="monospace", va="center"
)

# ── ROW_A: główne sterowanie ─────────────────────────────────────────

btn_start   = make_btn([0.01, ROW_A, 0.07, BTN_H], "▶ START",  C_START)
btn_stop    = make_btn([0.09, ROW_A, 0.07, BTN_H], "■ STOP",   C_STOP)
btn_autoall = make_btn([0.18, ROW_A, 0.08, BTN_H], "AUTO ALL", C_BLUE)
btn_clrcur  = make_btn([0.27, ROW_A, 0.08, BTN_H], "CLR CUR",  C_PURP)
btn_save    = make_btn([0.90, ROW_A, 0.08, BTN_H], "💾 SAVE",  C_GRAY)

# ── napis "per channel" ──────────────────────────────────────────────

fig.text(0.01, ROW_B + BTN_H + 0.003,
         "──── per channel ───────────────────────────────────────────────────────────────────",
         color="#1c2e42", fontsize=8, fontfamily="monospace", va="bottom")

# ── ROW_B: AUTO per kanał ────────────────────────────────────────────

auto_btns = []
CH_W = 0.185
CH_X = [0.01 + i * CH_W for i in range(N_CH)]

for i in range(N_CH):
    color_i = CH_DEF[i][2]
    b = make_btn([CH_X[i], ROW_B, CH_W - 0.005, BTN_H],
                 f"AUTO  {CH_DEF[i][0]}", C_BLUE, fsize=8, color=color_i)
    auto_btns.append(b)

# ── ROW_C: Y-center / Y-span / SET ──────────────────────────────────

txt_yc_list  = []
txt_ys_list  = []
set_btn_list = []

for i in range(N_CH):
    color_i = CH_DEF[i][2]
    x0 = CH_X[i]
    W  = CH_W - 0.005

    fig.text(x0 + W * 0.5, ROW_C + BTN_H + 0.003,
             CH_DEF[i][0], color=color_i, fontsize=7,
             fontfamily="monospace", ha="center", va="bottom")

    axc = plt.axes([x0,            ROW_C, W * 0.38, BTN_H])
    axs = plt.axes([x0 + W * 0.40, ROW_C, W * 0.38, BTN_H])
    axb = plt.axes([x0 + W * 0.80, ROW_C, W * 0.18, BTN_H])

    tc = TextBox(axc, "C:", initial=str(round(CH_DEF[i][3], 2)))
    ts = TextBox(axs, "S:", initial=str(round(CH_DEF[i][4], 2)))

    for tb in (tc, ts):
        tb.label.set_color(color_i)
        tb.label.set_fontsize(7)
        tb.text_disp.set_color("#ccddee")
        tb.text_disp.set_fontfamily("monospace")
        tb.text_disp.set_fontsize(8)

    b = Button(axb, "SET", color=C_BTN, hovercolor=C_BLUE)
    b.label.set_color(color_i)
    b.label.set_fontsize(8)
    b.label.set_fontfamily("monospace")

    txt_yc_list.append(tc)
    txt_ys_list.append(ts)
    set_btn_list.append(b)

# ── ROW_D: OVERLAY macierz ───────────────────────────────────────────

overlay_btns = {}
GUEST_H = 0.032

fig.text(0.005, ROW_D + GUEST_H * 2.2,
         "OVERLAY", color="#334455", fontsize=7,
         fontfamily="monospace", va="center", rotation=90)

for host in range(N_CH):
    color_h = CH_DEF[host][2]
    x0 = CH_X[host]
    W  = CH_W - 0.005
    fig.text(x0 + W * 0.5, ROW_D + (N_CH - 1) * GUEST_H + 0.008,
             f"▼ {CH_DEF[host][0]}", color=color_h,
             fontsize=7, fontfamily="monospace", ha="center")

guest_row = {host: 0 for host in range(N_CH)}

for guest in range(N_CH):
    for host in range(N_CH):
        if guest == host:
            continue
        color_g = CH_DEF[guest][2]
        x0  = CH_X[host]
        W   = CH_W - 0.005
        row = guest_row[host]
        y0  = ROW_D + row * GUEST_H

        ax_b = plt.axes([x0, y0, W, GUEST_H - 0.003])
        b = Button(ax_b, f"+ {CH_DEF[guest][0]}", color=C_OVL_OFF, hovercolor="#1a3050")
        b.label.set_color(color_g)
        b.label.set_fontsize(7)
        b.label.set_fontfamily("monospace")
        overlay_btns[(host, guest)] = b
        guest_row[host] += 1

# ═══════════════════════════════════════════════
# CALLBACKI
# ═══════════════════════════════════════════════

def cb_start(event):
    running_flag.set()
    print("[SCOPE] ▶ STARTED")

def cb_stop(event):
    running_flag.clear()
    print("[SCOPE] ■ STOPPED")

def cb_auto_all(event):
    shared_x["auto"]   = True
    shared_x["center"] = None
    for v in ch_y:
        v["auto"] = True

def cb_clr_cursor(event):
    cursor_x["active"] = False
    for cl in cur_lines:
        cl.set_visible(False)
    for cd in cur_dots:
        cd.set_visible(False)
    for clbl in cur_labels:
        clbl.set_visible(False)
    fig.canvas.draw_idle()

def make_auto_ch(idx):
    def cb(event):
        ch_y[idx]["auto"] = True
    return cb

def make_set_ch(idx):
    def cb(event):
        try:
            ch_y[idx]["center"] = float(txt_yc_list[idx].text)
            ch_y[idx]["span"]   = abs(float(txt_ys_list[idx].text))
            ch_y[idx]["auto"]   = False
        except Exception as e:
            print(f"[SET CH{idx}] błąd: {e}")
    return cb

def cb_save(event):
    with _buf_lock:
        n = len(buf["iL_A"])
        if n == 0:
            print("[SAVE] brak danych")
            return
        df = pd.DataFrame({
            "time": list(time_buf)[:n],
            **{k: list(buf[k]) for k in KEYS}
        })
    fn = f"pfc_log_{int(time.time())}.csv"
    df.to_csv(fn, index=False)
    print(f"[SAVE] → {fn}  ({n} próbek)")


def _make_gains_cb(tb_kp_w, tb_ki_w, status_text,
                   send_fn, label,
                   kp_range=(0.0, 10.0), ki_range=(0.0, 1000.0)):
    def cb(event):
        try:
            kp = float(tb_kp_w.text.strip())
            ki = float(tb_ki_w.text.strip())
        except ValueError:
            status_text.set_text("ERR: nieprawidłowa wartość (nie float)")
            status_text.set_color("#ff4444")
            fig.canvas.draw_idle()
            return

        if not (kp_range[0] <= kp <= kp_range[1]):
            status_text.set_text(f"ERR: Kp={kp:.4f} poza [{kp_range[0]}..{kp_range[1]}]")
            status_text.set_color("#ff8800")
            fig.canvas.draw_idle()
            return

        if not (ki_range[0] <= ki <= ki_range[1]):
            status_text.set_text(f"ERR: Ki={ki:.4f} poza [{ki_range[0]}..{ki_range[1]}]")
            status_text.set_color("#ff8800")
            fig.canvas.draw_idle()
            return

        status_text.set_text(f"Wysyłam [{label}]  Kp={kp:.5f}  Ki={ki:.5f} …")
        status_text.set_color("#ffdd44")
        fig.canvas.draw_idle()

        def _send():
            try:
                send_fn(kp, ki)
                status_text.set_text(f"✓ OK   Kp={kp:.5f}   Ki={ki:.5f}")
                status_text.set_color("#00f5a0")
            except Exception as e:
                status_text.set_text(f"ERR TX: {e}")
                status_text.set_color("#ff4444")
            fig.canvas.draw_idle()

        threading.Thread(target=_send, daemon=True).start()
    return cb


cb_send_gains = _make_gains_cb(
    tb_kp, tb_ki, gains_status,
    can_send_gains, "I-ctrl",
    kp_range=(0.0, 10.0), ki_range=(0.0, 1000.0)
)

cb_send_volt_gains = _make_gains_cb(
    tb_kp_v, tb_ki_v, volt_gains_status,
    can_send_volt_gains, "V-ctrl",
    kp_range=(0.0, 10.0), ki_range=(0.0, 1000.0)
)


def cb_send_vdc(event):
    try:
        vref = float(tb_vdc.text.strip())
    except ValueError:
        vdc_status.set_text("ERR: nieprawidłowa wartość")
        vdc_status.set_color("#ff4444")
        fig.canvas.draw_idle()
        return

    if not (0.0 <= vref <= 425.0):
        vdc_status.set_text(f"ERR: {vref:.1f}V poza zakresem [0..425]")
        vdc_status.set_color("#ff8800")
        fig.canvas.draw_idle()
        return

    vdc_status.set_text(f"Wysyłam Vdc_ref={vref:.1f}V …")
    vdc_status.set_color("#ffdd44")
    fig.canvas.draw_idle()

    def _send():
        try:
            can_send_vdc_ref(vref)
            vdc_status.set_text(f"✓ OK   Vdc_ref={vref:.1f}V")
            vdc_status.set_color("#00f5a0")
        except Exception as e:
            vdc_status.set_text(f"ERR TX: {e}")
            vdc_status.set_color("#ff4444")
        fig.canvas.draw_idle()

    threading.Thread(target=_send, daemon=True).start()


def make_overlay_cb(host, guest):
    def cb(event):
        s = overlay_map[host]
        if guest in s:
            s.discard(guest)
            overlay_btns[(host, guest)].ax.set_facecolor(C_OVL_OFF)
            print(f"[OVL] usuń CH{guest+1} z CH{host+1}")
        else:
            s.add(guest)
            overlay_btns[(host, guest)].ax.set_facecolor(C_OVL_ON)
            print(f"[OVL] nałóż CH{guest+1} na CH{host+1}")
        fig.canvas.draw_idle()
    return cb

# ── podpięcie callbacków ─────────────────────────────────────────────

btn_start.on_clicked(cb_start)
btn_stop.on_clicked(cb_stop)
btn_autoall.on_clicked(cb_auto_all)
btn_clrcur.on_clicked(cb_clr_cursor)
btn_save.on_clicked(cb_save)
btn_gains.on_clicked(cb_send_gains)
btn_vgains.on_clicked(cb_send_volt_gains)
btn_vdc.on_clicked(cb_send_vdc)

for i in range(N_CH):
    auto_btns[i].on_clicked(make_auto_ch(i))
    set_btn_list[i].on_clicked(make_set_ch(i))

for (host, guest), b in overlay_btns.items():
    b.on_clicked(make_overlay_cb(host, guest))

# ═══════════════════════════════════════════════
# LEGENDA SKRÓTÓW
# ═══════════════════════════════════════════════

fig.text(0.5, 0.01,
         "SCROLL = zoom X  |  SHIFT+SCROLL = zoom Y kanału  |  CTRL+SCROLL = pan X  |"
         "  LPM drag = pan  |  PPM klik/drag = kursor  |  ESC = usuń kursor",
         ha="center", color="#2a3a50", fontsize=7.5, fontfamily="monospace")

# ═══════════════════════════════════════════════
# START
# ═══════════════════════════════════════════════

threading.Thread(target=can_reader, daemon=True).start()
plt.show()