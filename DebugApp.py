#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np
import serial
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from PySide6 import QtCore, QtWidgets
from serial.tools import list_ports


@dataclass
class Sample:
    t: float
    x: float
    y: float
    th: float
    v: float
    w: float
    vl: float
    vr: float
    xd: Optional[float] = None
    yd: Optional[float] = None
    thd: Optional[float] = None
    vl_ref: Optional[float] = None
    vr_ref: Optional[float] = None
    v_ref: Optional[float] = None
    w_ref: Optional[float] = None
    pl: Optional[float] = None
    pr: Optional[float] = None
    pl_ref: Optional[float] = None
    pr_ref: Optional[float] = None


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def parse_line(line: str, *, track_width_in: Optional[float]) -> Optional[Sample]:
    line = line.strip()
    if not line:
        return None

    def vw_from_wheels(vl_: float, vr_: float) -> Tuple[float, float]:
        v_ = 0.5 * (vl_ + vr_)
        if track_width_in is None or abs(track_width_in) < 1e-9:
            w_ = 0.0
        else:
            w_ = (vr_ - vl_) / track_width_in
        return v_, w_

    if line.startswith("{") and line.endswith("}"):
        try:
            d = json.loads(line)

            def get_first(keys, default=None):
                for k in keys:
                    if k in d and d[k] is not None:
                        return d[k]
                return default

            t_raw = float(get_first(["t", "time"], 0.0))
            if t_raw > 1e6:
                t_s = t_raw * 1e-6
            elif t_raw > 1e3:
                t_s = t_raw * 1e-3
            else:
                t_s = t_raw

            x = float(d["x"])
            y = float(d["y"])
            th = float(get_first(["th", "theta", "heading"], 0.0))

            vl = float(get_first(["vl", "vL", "left"], 0.0))
            vr = float(get_first(["vr", "vR", "right"], 0.0))
            v = float(get_first(["v"], 0.5 * (vl + vr)))
            w = get_first(["w"], None)
            if w is None:
                v_auto, w_auto = vw_from_wheels(vl, vr)
                if "v" not in d:
                    v = v_auto
                w = w_auto
            else:
                w = float(w)

            xd = get_first(["xd", "x_d", "desired_x"], None)
            yd = get_first(["yd", "y_d", "desired_y"], None)
            thd = get_first(["thd", "theta_d", "heading_d", "desired_theta"], None)

            vl_ref = get_first(["vl_ref", "vL_ref", "vlRef"], None)
            vr_ref = get_first(["vr_ref", "vR_ref", "vrRef"], None)
            pl = get_first(["pl", "pL", "left_pos"], None)
            pr = get_first(["pr", "pR", "right_pos"], None)
            pl_ref = get_first(["pl_ref", "pL_ref", "plRef"], None)
            pr_ref = get_first(["pr_ref", "pR_ref", "prRef"], None)

            vl_ref_f = float(vl_ref) if vl_ref is not None else None
            vr_ref_f = float(vr_ref) if vr_ref is not None else None
            v_ref = float(d["v_ref"]) if ("v_ref" in d and d["v_ref"] is not None) else None
            w_ref = float(d["w_ref"]) if ("w_ref" in d and d["w_ref"] is not None) else None
            if v_ref is None and vl_ref_f is not None and vr_ref_f is not None:
                v_ref = 0.5 * (vl_ref_f + vr_ref_f)
            if w_ref is None and vl_ref_f is not None and vr_ref_f is not None and track_width_in:
                w_ref = (vr_ref_f - vl_ref_f) / track_width_in

            return Sample(
                t=t_s,
                x=x,
                y=y,
                th=_wrap_pi(th),
                v=v,
                w=w,
                vl=vl,
                vr=vr,
                xd=float(xd) if xd is not None else None,
                yd=float(yd) if yd is not None else None,
                thd=_wrap_pi(float(thd)) if thd is not None else None,
                vl_ref=vl_ref_f,
                vr_ref=vr_ref_f,
                v_ref=v_ref,
                w_ref=w_ref,
                pl=float(pl) if pl is not None else None,
                pr=float(pr) if pr is not None else None,
                pl_ref=float(pl_ref) if pl_ref is not None else None,
                pr_ref=float(pr_ref) if pr_ref is not None else None,
            )
        except Exception:
            return None

    parts = [p.strip() for p in line.split(",") if p.strip() != ""]
    if not parts:
        return None

    def is_number(s: str) -> bool:
        try:
            float(s)
            return True
        except Exception:
            return False

    if not is_number(parts[0]):
        if parts[0].upper() not in ("P", "POSE", "TELEM", "T"):
            return None
        parts = parts[1:]

    try:
        vals = list(map(float, parts))
    except Exception:
        return None

    try:
        if len(vals) == 6:
            t_ms, x, y, th, vl, vr = vals
            v, w = vw_from_wheels(vl, vr)
            return Sample(t_ms * 1e-3, x, y, _wrap_pi(th), v, w, vl, vr)

        if len(vals) == 8:
            t_ms, x, y, th, v, w, vl, vr = vals
            return Sample(t_ms * 1e-3, x, y, _wrap_pi(th), v, w, vl, vr)

        if len(vals) == 9:
            t_ms, x, y, th, vl, vr, pl, pr, _aux = vals
            v, w = vw_from_wheels(vl, vr)
            return Sample(t_ms * 1e-3, x, y, _wrap_pi(th), v, w, vl, vr, pl=pl, pr=pr)

        if len(vals) == 10:
            t_ms, x, y, th, vl, vr, pl, pr, pl_ref, pr_ref = vals
            v, w = vw_from_wheels(vl, vr)
            return Sample(t_ms * 1e-3, x, y, _wrap_pi(th), v, w, vl, vr, pl=pl, pr=pr, pl_ref=pl_ref, pr_ref=pr_ref)

        if len(vals) == 12:
            t_ms, x, y, th, xd, yd, thd, v, w, vl, vr, _aux = vals
            return Sample(t_ms * 1e-3, x, y, _wrap_pi(th), v, w, vl, vr, xd=xd, yd=yd, thd=_wrap_pi(thd))

        if len(vals) >= 15:
            t_ms, x, y, th, xd, yd, thd, vl, vr, vl_ref, vr_ref, pl, pr, pl_ref, pr_ref = vals[:15]
            v, w = vw_from_wheels(vl, vr)
            v_ref, w_ref = vw_from_wheels(vl_ref, vr_ref)
            return Sample(
                t_ms * 1e-3, x, y, _wrap_pi(th), v, w, vl, vr,
                xd=xd, yd=yd, thd=_wrap_pi(thd),
                vl_ref=vl_ref, vr_ref=vr_ref, v_ref=v_ref, w_ref=w_ref,
                pl=pl, pr=pr, pl_ref=pl_ref, pr_ref=pr_ref
            )
    except Exception:
        return None

    return None


class SerialWorker(QtCore.QObject):
    sample_rx = QtCore.Signal(object)
    raw_rx = QtCore.Signal(str)
    status_rx = QtCore.Signal(str)
    stats_rx = QtCore.Signal(int, int)

    def __init__(self):
        super().__init__()
        self._ser: Optional[serial.Serial] = None
        self._timer: Optional[QtCore.QTimer] = None
        self._stats_timer: Optional[QtCore.QTimer] = None
        self._paused = False
        self._track_width_in: Optional[float] = None
        self._prefix = "P"
        self._rx_chunks_accum = 0
        self._parse_err_accum = 0
        self._text_buffer = ""
        self._max_buffer = 16384

    @QtCore.Slot()
    def start(self):
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.poll_serial)
        self._timer.start(5)

        self._stats_timer = QtCore.QTimer(self)
        self._stats_timer.timeout.connect(self.emit_stats)
        self._stats_timer.start(1000)

    @QtCore.Slot(float, str)
    def configure(self, track_width_in: float, prefix: str):
        self._track_width_in = track_width_in if track_width_in > 0 else None
        self._prefix = prefix.strip().upper()

    @QtCore.Slot(str, int)
    def connect_port(self, port: str, baud: int):
        self.disconnect_port()
        try:
            self._ser = serial.Serial(port, baud, timeout=0.0)
            time.sleep(0.2)
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
            self._text_buffer = ""
            self.status_rx.emit(f"Connected: {port} @ {baud}")
        except Exception as e:
            self._ser = None
            self.status_rx.emit(f"Connect failed: {e}")

    @QtCore.Slot()
    def disconnect_port(self):
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._text_buffer = ""
        self.status_rx.emit("Disconnected")

    @QtCore.Slot(bool)
    def set_paused(self, paused: bool):
        self._paused = paused

    @QtCore.Slot(str)
    def send_line(self, text: str):
        if not self._ser:
            self.status_rx.emit("Not connected")
            return
        text = text.rstrip("\r\n") + "\n"
        try:
            self._ser.write(text.encode("utf-8", errors="replace"))
            self.status_rx.emit(f"Sent: {text.rstrip()}")
        except Exception as e:
            self.status_rx.emit(f"Send failed: {e}")

    @QtCore.Slot()
    def poll_serial(self):
        if not self._ser:
            return

        try:
            waiting = self._ser.in_waiting
        except Exception:
            waiting = 0

        if waiting <= 0:
            return

        try:
            raw = self._ser.read(waiting)
        except Exception:
            return

        if not raw:
            return

        self._rx_chunks_accum += 1
        chunk = raw.decode("utf-8", errors="replace")
        self._text_buffer += chunk

        if len(self._text_buffer) > self._max_buffer:
            self._text_buffer = self._text_buffer[-self._max_buffer:]

        self._extract_packets()

    def _extract_packets(self):
        buf = self._text_buffer

        while True:
            start = buf.find("{")
            if start < 0:
                tail = buf.strip()
                if tail:
                    self.raw_rx.emit(tail)
                self._text_buffer = ""
                return

            prefix = buf[:start].strip()
            if prefix:
                self.raw_rx.emit(prefix)

            depth = 0
            in_string = False
            escape = False
            end = None

            for i in range(start, len(buf)):
                c = buf[i]
                if in_string:
                    if escape:
                        escape = False
                    elif c == "\\":
                        escape = True
                    elif c == '"':
                        in_string = False
                    continue

                if c == '"':
                    in_string = True
                elif c == "{":
                    depth += 1
                elif c == "}":
                    depth -= 1
                    if depth == 0:
                        end = i
                        break

            if end is None:
                self._text_buffer = buf[start:]
                return

            packet = buf[start:end + 1]
            self.raw_rx.emit(packet)

            s = parse_line(packet, track_width_in=self._track_width_in)
            if s is None:
                self._parse_err_accum += 1
            elif not self._paused:
                self.sample_rx.emit(s)

            buf = buf[end + 1:]

    @QtCore.Slot()
    def emit_stats(self):
        self.stats_rx.emit(self._rx_chunks_accum, self._parse_err_accum)
        self._rx_chunks_accum = 0
        self._parse_err_accum = 0


class MplCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(constrained_layout=True)
        super().__init__(self.fig)
        self.setParent(parent)


def setup_dark_mpl():
    import matplotlib as mpl

    mpl.rcParams.update({
        "figure.facecolor": "#0b0b0b",
        "axes.facecolor": "#0b0b0b",
        "axes.edgecolor": "#666666",
        "axes.labelcolor": "#d8d8d8",
        "axes.titlecolor": "#ffffff",
        "xtick.color": "#c8c8c8",
        "ytick.color": "#c8c8c8",
        "grid.color": "#5a5a5a",
        "grid.alpha": 0.22,
        "text.color": "#ffffff",
        "legend.facecolor": "#141414",
        "legend.edgecolor": "#444444",
        "savefig.facecolor": "#0b0b0b",
        "savefig.edgecolor": "#0b0b0b",
        "font.size": 9,
        "toolbar": "toolbar2",
    })


class RobotDebugUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Serial Debug UI")

        setup_dark_mpl()

        self.max_history = 20000
        self.window_points = 1000
        self.heading_deg = False
        self.arrow_len = 3.0
        self.console_max_lines = 500
        self.line_width = 0.8
        self.heading_line_width = 0.9

        self.sample_idx: Deque[int] = deque(maxlen=self.max_history)
        self.ts: Deque[float] = deque(maxlen=self.max_history)
        self.xs: Deque[float] = deque(maxlen=self.max_history)
        self.ys: Deque[float] = deque(maxlen=self.max_history)
        self.ths: Deque[float] = deque(maxlen=self.max_history)
        self.vs: Deque[float] = deque(maxlen=self.max_history)
        self.ws: Deque[float] = deque(maxlen=self.max_history)
        self.vls: Deque[float] = deque(maxlen=self.max_history)
        self.vrs: Deque[float] = deque(maxlen=self.max_history)
        self.pls: Deque[float] = deque(maxlen=self.max_history)
        self.prs: Deque[float] = deque(maxlen=self.max_history)

        self.xds: Deque[float] = deque(maxlen=self.max_history)
        self.yds: Deque[float] = deque(maxlen=self.max_history)
        self.thds: Deque[float] = deque(maxlen=self.max_history)
        self.vs_ref: Deque[float] = deque(maxlen=self.max_history)
        self.ws_ref: Deque[float] = deque(maxlen=self.max_history)
        self.vls_ref: Deque[float] = deque(maxlen=self.max_history)
        self.vrs_ref: Deque[float] = deque(maxlen=self.max_history)
        self.pls_ref: Deque[float] = deque(maxlen=self.max_history)
        self.prs_ref: Deque[float] = deque(maxlen=self.max_history)

        self.exs: Deque[float] = deque(maxlen=self.max_history)
        self.eys: Deque[float] = deque(maxlen=self.max_history)
        self.eths: Deque[float] = deque(maxlen=self.max_history)

        self._next_index = 0
        self._dirty = False
        self._log_fp = None
        self._log_writer = None
        self._samples_per_s = 0
        self._samples_counter = 0
        self._samples_counter_last = time.monotonic()

        self._build_ui()
        self._setup_worker()
        self.refresh_ports()

        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self.redraw)
        self.ui_timer.start(50)

    def _setup_worker(self):
        self.worker_thread = QtCore.QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.worker_thread)

        self.worker_thread.started.connect(self.worker.start)
        self.worker.sample_rx.connect(self.on_sample)
        self.worker.raw_rx.connect(self.on_raw_line)
        self.worker.status_rx.connect(self.set_status)
        self.worker.stats_rx.connect(self.on_stats)
        self.worker_thread.start()

    def _canvas_with_toolbar(self, canvas: MplCanvas) -> QtWidgets.QWidget:
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        toolbar = NavigationToolbar(canvas, container)
        toolbar.setStyleSheet("""
            QWidget {
                background-color: #111111;
                color: #dddddd;
            }
            QToolButton {
                background-color: #181818;
                color: #dddddd;
                border: 1px solid #333333;
                padding: 2px;
            }
            QToolButton:hover {
                background-color: #222222;
            }
        """)
        layout.addWidget(toolbar)
        layout.addWidget(canvas, 1)
        return container

    def _build_ui(self):
        self.setCentralWidget(QtWidgets.QWidget())
        root = QtWidgets.QVBoxLayout(self.centralWidget())
        root.addWidget(self._build_controls())

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_pose_tab(), "Pose")
        self.tabs.addTab(self._build_wheel_tab(), "Wheels")
        self.tabs.addTab(self._build_console_tab(), "Serial Console")
        root.addWidget(self.tabs, 1)

        self._build_status()

    def _build_controls(self):
        w = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(w)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(4)

        row1 = QtWidgets.QHBoxLayout()
        row2 = QtWidgets.QHBoxLayout()

        self.port_combo = QtWidgets.QComboBox()
        self.refresh_btn = QtWidgets.QPushButton("Refresh")

        self.baud_combo = QtWidgets.QComboBox()
        for b in [115200, 230400, 460800, 921600]:
            self.baud_combo.addItem(str(b), b)
        self.baud_combo.setCurrentText("115200")

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)

        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)
        self.clear_btn = QtWidgets.QPushButton("Clear")

        self.track_width = QtWidgets.QDoubleSpinBox()
        self.track_width.setRange(0.0, 1000.0)
        self.track_width.setDecimals(3)
        self.track_width.setValue(9.375)
        self.track_width.setSuffix(" in")

        self.window_spin = QtWidgets.QSpinBox()
        self.window_spin.setRange(20, self.max_history)
        self.window_spin.setSingleStep(100)
        self.window_spin.setValue(self.window_points)

        self.prefix_edit = QtWidgets.QLineEdit("P")
        self.prefix_edit.setMaximumWidth(60)

        self.heading_unit = QtWidgets.QComboBox()
        self.heading_unit.addItems(["rad", "deg"])

        self.log_btn = QtWidgets.QPushButton("Start Log…")
        self.log_btn.setCheckable(True)

        for item in [
            QtWidgets.QLabel("Port:"), self.port_combo, self.refresh_btn,
            QtWidgets.QLabel("Baud:"), self.baud_combo,
            self.connect_btn, self.disconnect_btn, self.pause_btn, self.clear_btn,
        ]:
            row1.addWidget(item)

        row1.addStretch(1)

        for item in [
            QtWidgets.QLabel("Track width:"), self.track_width,
            QtWidgets.QLabel("Window:"), self.window_spin,
            QtWidgets.QLabel("Prefix:"), self.prefix_edit,
            QtWidgets.QLabel("Heading:"), self.heading_unit,
            self.log_btn,
        ]:
            row2.addWidget(item)

        row2.addStretch(1)

        outer.addLayout(row1)
        outer.addLayout(row2)

        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.pause_btn.toggled.connect(self.on_pause_toggled)
        self.clear_btn.clicked.connect(self.clear_buffers)
        self.heading_unit.currentTextChanged.connect(self.on_heading_unit_changed)
        self.log_btn.toggled.connect(self.on_log_toggled)
        self.window_spin.valueChanged.connect(self.on_window_changed)

        return w

    def _style_axes(self, ax, title: str, xlabel: str, ylabel: str):
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True, linewidth=0.5)
        for spine in ax.spines.values():
            spine.set_color("#666666")

    def _build_pose_tab(self):
        container = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(container)
        outer.setContentsMargins(0, 0, 0, 0)

        self.pose_canvas = MplCanvas()
        gs = self.pose_canvas.fig.add_gridspec(
            3, 2,
            width_ratios=[1.7, 1.0],
            height_ratios=[1.0, 1.0, 0.48]
        )

        self.ax_xy = self.pose_canvas.fig.add_subplot(gs[:, 0])
        self.ax_th = self.pose_canvas.fig.add_subplot(gs[0, 1])
        self.ax_v = self.pose_canvas.fig.add_subplot(gs[1, 1])
        self.ax_err = self.pose_canvas.fig.add_subplot(gs[2, 1])

        self._style_axes(self.ax_xy, "World XY trajectory", "x [in]", "y [in]")
        # self.ax_xy.set_aspect("equal", adjustable="datalim")

        self._style_axes(self.ax_th, "Heading", "sample", "heading [rad]")
        self._style_axes(self.ax_v, "Chassis linear velocity", "sample", "v [in/s]")
        self._style_axes(self.ax_err, "Pose error", "sample", "error")

        (self.xy_actual_line,) = self.ax_xy.plot([], [], linewidth=self.line_width, label="actual path")
        (self.xy_des_line,) = self.ax_xy.plot([], [], "--", linewidth=self.line_width, label="desired path")
        (self.xy_actual_pt,) = self.ax_xy.plot([], [], "o", markersize=3.0, label="_nolegend_")
        (self.xy_des_pt,) = self.ax_xy.plot([], [], "x", markersize=4.0, mew=0.8, label="_nolegend_")
        (self.xy_heading_line,) = self.ax_xy.plot([], [], linewidth=self.heading_line_width, label="actual heading")
        (self.xy_heading_des_line,) = self.ax_xy.plot([], [], "--", linewidth=self.heading_line_width, label="desired heading")
        self.ax_xy.legend(loc="upper left")

        (self.th_line,) = self.ax_th.plot([], [], linewidth=self.line_width, label="heading")
        self.ax_th.legend(loc="upper left")

        (self.v_line,) = self.ax_v.plot([], [], linewidth=self.line_width, label="actual")
        (self.v_ref_line,) = self.ax_v.plot([], [], "--", linewidth=self.line_width, label="desired")
        self.ax_v.legend(loc="upper left")

        (self.ex_line,) = self.ax_err.plot([], [], linewidth=self.line_width, label="x error")
        (self.ey_line,) = self.ax_err.plot([], [], linewidth=self.line_width, label="y error")
        (self.eth_line,) = self.ax_err.plot([], [], "--", linewidth=self.line_width, label="heading error")
        self.ax_err.legend(loc="upper left")

        self.angvel_canvas = MplCanvas()
        self.ax_w = self.angvel_canvas.fig.add_subplot(111)
        self._style_axes(self.ax_w, "Chassis angular velocity", "sample", "ω [rad/s]")
        (self.w_line,) = self.ax_w.plot([], [], linewidth=self.line_width, label="actual")
        (self.w_ref_line,) = self.ax_w.plot([], [], "--", linewidth=self.line_width, label="desired")
        self.ax_w.legend(loc="upper left")

        split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        split.addWidget(self._canvas_with_toolbar(self.pose_canvas))
        split.addWidget(self._canvas_with_toolbar(self.angvel_canvas))
        split.setStretchFactor(0, 5)
        split.setStretchFactor(1, 2)

        outer.addWidget(split, 1)
        return container

    def _build_wheel_tab(self):
        container = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(container)
        outer.setContentsMargins(0, 0, 0, 0)

        self.wheel_canvas = MplCanvas()
        gs = self.wheel_canvas.fig.add_gridspec(2, 2, height_ratios=[1.0, 1.0])

        self.ax_wheel_vel = self.wheel_canvas.fig.add_subplot(gs[0, :])
        self.ax_left_pos = self.wheel_canvas.fig.add_subplot(gs[1, 0])
        self.ax_right_pos = self.wheel_canvas.fig.add_subplot(gs[1, 1])

        self._style_axes(self.ax_wheel_vel, "Wheel velocities", "sample", "wheel v [in/s]")
        self._style_axes(self.ax_left_pos, "Left wheel position", "sample", "position")
        self._style_axes(self.ax_right_pos, "Right wheel position", "sample", "position")

        (self.vl_line,) = self.ax_wheel_vel.plot([], [], linewidth=self.line_width, label="left actual")
        (self.vl_ref_line,) = self.ax_wheel_vel.plot([], [], "--", linewidth=self.line_width, label="left desired")
        (self.vr_line,) = self.ax_wheel_vel.plot([], [], linewidth=self.line_width, label="right actual")
        (self.vr_ref_line,) = self.ax_wheel_vel.plot([], [], "--", linewidth=self.line_width, label="right desired")
        self.ax_wheel_vel.legend(loc="upper left")

        (self.left_pos_line,) = self.ax_left_pos.plot([], [], linewidth=self.line_width, label="left actual")
        (self.left_pos_ref_line,) = self.ax_left_pos.plot([], [], "--", linewidth=self.line_width, label="left desired")
        self.ax_left_pos.legend(loc="upper left")

        (self.right_pos_line,) = self.ax_right_pos.plot([], [], linewidth=self.line_width, label="right actual")
        (self.right_pos_ref_line,) = self.ax_right_pos.plot([], [], "--", linewidth=self.line_width, label="right desired")
        self.ax_right_pos.legend(loc="upper left")

        outer.addWidget(self._canvas_with_toolbar(self.wheel_canvas), 1)
        return container

    def _build_console_tab(self):
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)

        self.console_view = QtWidgets.QPlainTextEdit()
        self.console_view.setReadOnly(True)
        self.console_view.setMaximumBlockCount(self.console_max_lines)

        self.console_input = QtWidgets.QLineEdit()
        self.console_input.setPlaceholderText("Enter command and press Enter or Send")

        send_row = QtWidgets.QHBoxLayout()
        self.send_btn = QtWidgets.QPushButton("Send")
        self.clear_console_btn = QtWidgets.QPushButton("Clear Console")
        send_row.addWidget(self.console_input, 1)
        send_row.addWidget(self.send_btn)
        send_row.addWidget(self.clear_console_btn)

        layout.addWidget(self.console_view, 1)
        layout.addLayout(send_row)

        self.send_btn.clicked.connect(self.on_send_command)
        self.console_input.returnPressed.connect(self.on_send_command)
        self.clear_console_btn.clicked.connect(self.console_view.clear)
        return container

    def _build_status(self):
        self.status = QtWidgets.QStatusBar()
        self.setStatusBar(self.status)
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.stats_label = QtWidgets.QLabel("RX: 0 chunks/s | samples: 0 sps | parse err: 0/s")
        self.status.addWidget(self.status_label, 1)
        self.status.addPermanentWidget(self.stats_label)

    def set_status(self, msg: str):
        self.status_label.setText(msg)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list(list_ports.comports())

        def score(p):
            desc = (p.description or "").lower()
            manu = (p.manufacturer or "").lower()
            s = 0
            if "arduino" in desc or "arduino" in manu:
                s += 10
            if "usb" in desc and ("serial" in desc or "cdc" in desc):
                s += 5
            return -s

        for p in sorted(ports, key=score):
            self.port_combo.addItem(f"{p.device} — {p.description}", p.device)
        if self.port_combo.count() == 0:
            self.port_combo.addItem("(no ports found)", "")

    def current_port(self) -> str:
        return self.port_combo.currentData() or ""

    def current_baud(self) -> int:
        return int(self.baud_combo.currentData())

    def on_window_changed(self, value: int):
        self.window_points = int(value)
        self._dirty = True

    def _clear_plot_lines(self):
        for line in [
            self.xy_actual_line, self.xy_des_line, self.xy_actual_pt, self.xy_des_pt,
            self.xy_heading_line, self.xy_heading_des_line, self.th_line,
            self.v_line, self.v_ref_line, self.w_line, self.w_ref_line,
            self.ex_line, self.ey_line, self.eth_line,
            self.vl_line, self.vl_ref_line, self.vr_line, self.vr_ref_line,
            self.left_pos_line, self.left_pos_ref_line, self.right_pos_line, self.right_pos_ref_line,
        ]:
            line.set_data([], [])

        for ax in [
            self.ax_xy, self.ax_th, self.ax_v, self.ax_w, self.ax_err,
            self.ax_wheel_vel, self.ax_left_pos, self.ax_right_pos
        ]:
            ax.relim()
            ax.autoscale_view()

        self.pose_canvas.draw_idle()
        self.angvel_canvas.draw_idle()
        self.wheel_canvas.draw_idle()

    def clear_buffers(self):
        for dq in [
            self.sample_idx, self.ts, self.xs, self.ys, self.ths, self.vs, self.ws, self.vls, self.vrs, self.pls, self.prs,
            self.xds, self.yds, self.thds, self.vs_ref, self.ws_ref, self.vls_ref, self.vrs_ref, self.pls_ref, self.prs_ref,
            self.exs, self.eys, self.eths,
        ]:
            dq.clear()

        self._next_index = 0
        self.console_view.clear()
        self._clear_plot_lines()

    def on_connect(self):
        port = self.current_port()
        if not port:
            self.set_status("No port selected")
            return

        QtCore.QMetaObject.invokeMethod(
            self.worker,
            "configure",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(float, float(self.track_width.value())),
            QtCore.Q_ARG(str, self.prefix_edit.text()),
        )

        QtCore.QMetaObject.invokeMethod(
            self.worker,
            "connect_port",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(str, port),
            QtCore.Q_ARG(int, self.current_baud()),
        )

        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)

    def on_disconnect(self):
        QtCore.QMetaObject.invokeMethod(self.worker, "disconnect_port", QtCore.Qt.QueuedConnection)
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

    def on_pause_toggled(self, paused: bool):
        QtCore.QMetaObject.invokeMethod(
            self.worker,
            "set_paused",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(bool, paused),
        )
        self.pause_btn.setText("Resume" if paused else "Pause")

    def on_heading_unit_changed(self, unit: str):
        self.heading_deg = (unit.lower() == "deg")
        self.ax_th.set_ylabel("heading [deg]" if self.heading_deg else "heading [rad]")
        self.ax_err.set_ylabel("error (e_th in deg)" if self.heading_deg else "error")
        self._dirty = True

    def on_log_toggled(self, enabled: bool):
        if enabled:
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save CSV Log", "robot_telemetry.csv", "CSV Files (*.csv)"
            )
            if not path:
                self.log_btn.setChecked(False)
                return
            try:
                self._log_fp = open(path, "w", newline="", buffering=1)
                self._log_writer = csv.writer(self._log_fp)
                self._log_writer.writerow([
                    "sample", "t_s", "x", "y", "th", "v", "w", "vl", "vr", "pl", "pr",
                    "xd", "yd", "thd", "vl_ref", "vr_ref", "v_ref", "w_ref", "pl_ref", "pr_ref",
                ])
                self.log_btn.setText("Stop Log")
            except Exception as e:
                self.set_status(f"Log open failed: {e}")
                self.log_btn.setChecked(False)
        else:
            self.log_btn.setText("Start Log…")
            try:
                if self._log_fp:
                    self._log_fp.close()
            except Exception:
                pass
            self._log_fp = None
            self._log_writer = None

    def on_send_command(self):
        text = self.console_input.text().strip()
        if not text:
            return
        self.console_view.appendPlainText(f"> {text}")
        QtCore.QMetaObject.invokeMethod(
            self.worker,
            "send_line",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(str, text),
        )
        self.console_input.clear()

    @QtCore.Slot(str)
    def on_raw_line(self, line: str):
        self.console_view.appendPlainText(line)

    @QtCore.Slot(object)
    def on_sample(self, s: Sample):
        idx = self._next_index
        self._next_index += 1

        self.sample_idx.append(idx)
        self.ts.append(s.t)
        self.xs.append(s.x)
        self.ys.append(s.y)
        self.ths.append(_wrap_pi(s.th))
        self.vs.append(s.v)
        self.ws.append(s.w)
        self.vls.append(s.vl)
        self.vrs.append(s.vr)
        self.pls.append(np.nan if s.pl is None else s.pl)
        self.prs.append(np.nan if s.pr is None else s.pr)

        self.xds.append(np.nan if s.xd is None else s.xd)
        self.yds.append(np.nan if s.yd is None else s.yd)
        self.thds.append(np.nan if s.thd is None else _wrap_pi(s.thd))
        self.vs_ref.append(np.nan if s.v_ref is None else s.v_ref)
        self.ws_ref.append(np.nan if s.w_ref is None else s.w_ref)
        self.vls_ref.append(np.nan if s.vl_ref is None else s.vl_ref)
        self.vrs_ref.append(np.nan if s.vr_ref is None else s.vr_ref)
        self.pls_ref.append(np.nan if s.pl_ref is None else s.pl_ref)
        self.prs_ref.append(np.nan if s.pr_ref is None else s.pr_ref)

        if s.xd is not None and s.yd is not None and s.thd is not None:
            self.exs.append(s.xd - s.x)
            self.eys.append(s.yd - s.y)
            self.eths.append(_wrap_pi(s.thd - s.th))
        else:
            self.exs.append(np.nan)
            self.eys.append(np.nan)
            self.eths.append(np.nan)

        if self._log_writer:
            self._log_writer.writerow([
                idx, s.t, s.x, s.y, s.th, s.v, s.w, s.vl, s.vr,
                s.pl if s.pl is not None else "",
                s.pr if s.pr is not None else "",
                s.xd if s.xd is not None else "",
                s.yd if s.yd is not None else "",
                s.thd if s.thd is not None else "",
                s.vl_ref if s.vl_ref is not None else "",
                s.vr_ref if s.vr_ref is not None else "",
                s.v_ref if s.v_ref is not None else "",
                s.w_ref if s.w_ref is not None else "",
                s.pl_ref if s.pl_ref is not None else "",
                s.pr_ref if s.pr_ref is not None else "",
            ])

        self._samples_counter += 1
        now = time.monotonic()
        if now - self._samples_counter_last >= 1.0:
            self._samples_per_s = self._samples_counter
            self._samples_counter = 0
            self._samples_counter_last = now

        self._dirty = True

    @QtCore.Slot(int, int)
    def on_stats(self, chunks: int, parse_err: int):
        self.stats_label.setText(
            f"RX: {chunks} chunks/s | samples: {self._samples_per_s} sps | parse err: {parse_err}/s"
        )

    def _last_window(self, dq: Deque[float] | Deque[int]) -> np.ndarray:
        arr = np.asarray(dq, dtype=float)
        if arr.size <= self.window_points:
            return arr
        return arr[-self.window_points:]

    def _set_scroll_xlim(self, ax, idx_arr: np.ndarray):
        if idx_arr.size == 0:
            return
        left = idx_arr[0]
        right = idx_arr[-1] if idx_arr.size > 1 else idx_arr[0] + 1
        if right <= left:
            right = left + 1
        ax.set_xlim(left, right)

    def _autoscale_y(self, ax, arrays: list[np.ndarray], pad_frac: float = 0.08):
        vals = []
        for a in arrays:
            if a.size:
                finite = a[np.isfinite(a)]
                if finite.size:
                    vals.append(finite)

        if not vals:
            return

        allv = np.concatenate(vals)
        ymin = float(np.min(allv))
        ymax = float(np.max(allv))
        if math.isclose(ymin, ymax):
            d = 1.0 if abs(ymin) < 1e-9 else abs(ymin) * 0.1
            ymin -= d
            ymax += d
        else:
            pad = (ymax - ymin) * pad_frac
            ymin -= pad
            ymax += pad
        ax.set_ylim(ymin, ymax)

    def _autoscale_xy_all_points(self):
        xs_all = np.asarray(self.xs, dtype=float)
        ys_all = np.asarray(self.ys, dtype=float)
        xds_all = np.asarray(self.xds, dtype=float)
        yds_all = np.asarray(self.yds, dtype=float)

        valid_actual = np.isfinite(xs_all) & np.isfinite(ys_all)
        valid_des = np.isfinite(xds_all) & np.isfinite(yds_all)

        pieces_x = []
        pieces_y = []

        if np.any(valid_actual):
            pieces_x.append(xs_all[valid_actual])
            pieces_y.append(ys_all[valid_actual])
        if np.any(valid_des):
            pieces_x.append(xds_all[valid_des])
            pieces_y.append(yds_all[valid_des])

        if not pieces_x:
            return

        all_x = np.concatenate(pieces_x)
        all_y = np.concatenate(pieces_y)

        xmin, xmax = float(np.min(all_x)), float(np.max(all_x))
        ymin, ymax = float(np.min(all_y)), float(np.max(all_y))

        if math.isclose(xmin, xmax):
            xmin -= 1.0
            xmax += 1.0
        if math.isclose(ymin, ymax):
            ymin -= 1.0
            ymax += 1.0

        pad_x = (xmax - xmin) * 0.08
        pad_y = (ymax - ymin) * 0.08
        self.ax_xy.set_xlim(xmin - pad_x, xmax + pad_x)
        self.ax_xy.set_ylim(ymin - pad_y, ymax + pad_y)

    def redraw(self):
        if not self._dirty or len(self.sample_idx) < 1:
            return

        idx = self._last_window(self.sample_idx)

        xs = self._last_window(self.xs)
        ys = self._last_window(self.ys)
        ths = self._last_window(self.ths)
        vs = self._last_window(self.vs)
        ws = self._last_window(self.ws)
        vls = self._last_window(self.vls)
        vrs = self._last_window(self.vrs)
        pls = self._last_window(self.pls)
        prs = self._last_window(self.prs)

        xds = self._last_window(self.xds)
        yds = self._last_window(self.yds)
        thds = self._last_window(self.thds)
        vs_ref = self._last_window(self.vs_ref)
        ws_ref = self._last_window(self.ws_ref)
        vls_ref = self._last_window(self.vls_ref)
        vrs_ref = self._last_window(self.vrs_ref)
        pls_ref = self._last_window(self.pls_ref)
        prs_ref = self._last_window(self.prs_ref)
        exs = self._last_window(self.exs)
        eys = self._last_window(self.eys)
        eths = self._last_window(self.eths)

        self.xy_actual_line.set_data(xs, ys)
        if xs.size and ys.size:
            self.xy_actual_pt.set_data([xs[-1]], [ys[-1]])
            th = float(ths[-1])
            x0, y0 = float(xs[-1]), float(ys[-1])
            self.xy_heading_line.set_data(
                [x0, x0 + self.arrow_len * math.cos(th)],
                [y0, y0 + self.arrow_len * math.sin(th)]
            )
        else:
            self.xy_actual_pt.set_data([], [])
            self.xy_heading_line.set_data([], [])

        valid_des = np.isfinite(xds) & np.isfinite(yds)
        self.xy_des_line.set_data(xds[valid_des], yds[valid_des])
        if np.any(valid_des):
            xd_last = xds[valid_des][-1]
            yd_last = yds[valid_des][-1]
            self.xy_des_pt.set_data([xd_last], [yd_last])

            valid_thd = np.isfinite(thds)
            if np.any(valid_thd):
                thd_last = thds[valid_thd][-1]
                self.xy_heading_des_line.set_data(
                    [xd_last, xd_last + self.arrow_len * math.cos(thd_last)],
                    [yd_last, yd_last + self.arrow_len * math.sin(thd_last)]
                )
            else:
                self.xy_heading_des_line.set_data([], [])
        else:
            self.xy_des_pt.set_data([], [])
            self.xy_heading_des_line.set_data([], [])

        self._autoscale_xy_all_points()

        th_plot = np.degrees(ths) if self.heading_deg else ths
        self.th_line.set_data(idx, th_plot)
        self._set_scroll_xlim(self.ax_th, idx)
        self._autoscale_y(self.ax_th, [th_plot])

        self.v_line.set_data(idx, vs)
        self.v_ref_line.set_data(idx, vs_ref)
        self._set_scroll_xlim(self.ax_v, idx)
        self._autoscale_y(self.ax_v, [vs, vs_ref])

        self.w_line.set_data(idx, ws)
        self.w_ref_line.set_data(idx, ws_ref)
        self._set_scroll_xlim(self.ax_w, idx)
        self._autoscale_y(self.ax_w, [ws, ws_ref])

        eth_plot = np.degrees(eths) if self.heading_deg else eths
        self.ex_line.set_data(idx, exs)
        self.ey_line.set_data(idx, eys)
        self.eth_line.set_data(idx, eth_plot)
        self._set_scroll_xlim(self.ax_err, idx)
        self._autoscale_y(self.ax_err, [exs, eys, eth_plot])

        self.vl_line.set_data(idx, vls)
        self.vl_ref_line.set_data(idx, vls_ref)
        self.vr_line.set_data(idx, vrs)
        self.vr_ref_line.set_data(idx, vrs_ref)
        self._set_scroll_xlim(self.ax_wheel_vel, idx)
        self._autoscale_y(self.ax_wheel_vel, [vls, vls_ref, vrs, vrs_ref])

        self.left_pos_line.set_data(idx, pls)
        self.left_pos_ref_line.set_data(idx, pls_ref)
        self._set_scroll_xlim(self.ax_left_pos, idx)
        self._autoscale_y(self.ax_left_pos, [pls, pls_ref])

        self.right_pos_line.set_data(idx, prs)
        self.right_pos_ref_line.set_data(idx, prs_ref)
        self._set_scroll_xlim(self.ax_right_pos, idx)
        self._autoscale_y(self.ax_right_pos, [prs, prs_ref])

        self.pose_canvas.draw_idle()
        self.angvel_canvas.draw_idle()
        self.wheel_canvas.draw_idle()
        self._dirty = False

    def closeEvent(self, event):
        try:
            QtCore.QMetaObject.invokeMethod(self.worker, "disconnect_port", QtCore.Qt.QueuedConnection)
        except Exception:
            pass
        try:
            self.worker_thread.quit()
            self.worker_thread.wait(1000)
        except Exception:
            pass
        try:
            if self._log_fp:
                self._log_fp.close()
        except Exception:
            pass
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)

    app.setStyleSheet("""
        QWidget {
            background-color: #090909;
            color: #dddddd;
        }
        QMainWindow, QTabWidget, QPlainTextEdit, QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
            background-color: #111111;
            color: #dddddd;
        }
        QPushButton {
            background-color: #181818;
            border: 1px solid #444444;
            padding: 5px 10px;
        }
        QPushButton:checked {
            background-color: #222222;
        }
        QStatusBar {
            background-color: #111111;
            color: #dddddd;
        }
    """)

    win = RobotDebugUI()
    win.resize(1550, 980)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())