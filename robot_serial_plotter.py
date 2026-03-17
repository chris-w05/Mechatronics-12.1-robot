#!/usr/bin/env python3
"""
robot_debug_ui.py

Qt + pyqtgraph UI for live serial telemetry debugging (diff-drive robot).

Features:
- Port/baud selection + connect/disconnect
- Pause/resume plotting
- CSV logging from UI
- Track width + prefix config
- Fast plots: XY trajectory (actual+desired) w/ heading arrow,
  heading vs time, chassis v/w, wheels vL/vR (+ optional refs)

Telemetry formats supported: same as your original script (CSV + JSON).
"""

from __future__ import annotations

import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional, Deque, Tuple

from collections import deque

import numpy as np
import serial  # pip install pyserial
from serial.tools import list_ports

from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg


# ----------------------------
# Telemetry model + parsing
# ----------------------------

@dataclass
class Sample:
    # Actual
    t: float  # seconds
    x: float  # inches
    y: float  # inches
    th: float  # radians
    v: float  # in/s
    w: float  # rad/s
    vl: float  # in/s
    vr: float  # in/s

    # Desired / reference pose (optional)
    xd: Optional[float] = None
    yd: Optional[float] = None
    thd: Optional[float] = None

    # Reference velocities (optional)
    vl_ref: Optional[float] = None
    vr_ref: Optional[float] = None
    v_ref: Optional[float] = None
    w_ref: Optional[float] = None


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

    # JSON
    if line.startswith("{") and line.endswith("}"):
        try:
            d = json.loads(line)

            def get_first(keys, default=None):
                for k in keys:
                    if k in d and d[k] is not None:
                        return d[k]
                return default

            t_raw = float(d.get("t", d.get("time", 0.0)))
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

            v = float(d.get("v", 0.5 * (vl + vr)))
            if "w" in d and d["w"] is not None:
                w = float(d["w"])
            else:
                v_, w_ = vw_from_wheels(vl, vr)
                w = w_
                if "v" not in d:
                    v = v_

            xd = get_first(["xd", "x_d", "xdes", "x_ref", "xRef"], None)
            yd = get_first(["yd", "y_d", "ydes", "y_ref", "yRef"], None)
            thd = get_first(
                ["thd", "theta_d", "heading_d", "th_ref", "thetaRef", "headingRef"],
                None,
            )

            vl_ref = get_first(["vl_ref", "vL_ref", "vlRef", "vLRef"], None)
            vr_ref = get_first(["vr_ref", "vR_ref", "vrRef", "vRRef"], None)

            vl_ref_f = float(vl_ref) if vl_ref is not None else None
            vr_ref_f = float(vr_ref) if vr_ref is not None else None

            v_ref = float(d["v_ref"]) if ("v_ref" in d and d["v_ref"] is not None) else None
            w_ref = float(d["w_ref"]) if ("w_ref" in d and d["w_ref"] is not None) else None

            if v_ref is None and vl_ref_f is not None and vr_ref_f is not None:
                v_ref = 0.5 * (vl_ref_f + vr_ref_f)
            if w_ref is None and vl_ref_f is not None and vr_ref_f is not None:
                if track_width_in is not None and abs(track_width_in) > 1e-9:
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
            )
        except Exception:
            return None

    # CSV (allow optional prefix token like "P")
    parts = [p.strip() for p in line.split(",") if p.strip() != ""]
    if not parts:
        return None

    # skip non-telemetry prefixes
    def is_number(s: str) -> bool:
        try:
            float(s)
            return True
        except Exception:
            return False

    if not is_number(parts[0]):
        prefix = parts[0].upper()
        if prefix not in ("P", "POSE", "TELEM", "T"):
            return None
        parts = parts[1:]

    if len(parts) < 6:
        return None

    try:
        # 1) t,x,y,th,vl,vr
        if len(parts) == 6:
            t_ms, x, y, th, vl, vr = map(float, parts)
            t_s = t_ms * 1e-3
            v, w = vw_from_wheels(vl, vr)
            return Sample(t_s, x, y, _wrap_pi(th), v, w, vl, vr)

        # 2) t,x,y,th,v,w,vl,vr
        if len(parts) == 8:
            t_ms, x, y, th, v, w, vl, vr = map(float, parts)
            t_s = t_ms * 1e-3
            return Sample(t_s, x, y, _wrap_pi(th), v, w, vl, vr)

        # 3) t,x,y,th,xd,yd,thd,vl,vr
        if len(parts) == 9:
            t_ms, x, y, th, xd, yd, thd, vl, vr = map(float, parts)
            t_s = t_ms * 1e-3
            v, w = vw_from_wheels(vl, vr)
            return Sample(t_s, x, y, _wrap_pi(th), v, w, vl, vr, xd=xd, yd=yd, thd=_wrap_pi(thd))

        # 4) t,x,y,th,xd,yd,thd,vl,vr,vl_ref,vr_ref
        if len(parts) == 11:
            t_ms, x, y, th, xd, yd, thd, vl, vr, vl_ref, vr_ref = map(float, parts)
            t_s = t_ms * 1e-3
            v, w = vw_from_wheels(vl, vr)
            v_ref, w_ref = vw_from_wheels(vl_ref, vr_ref)
            return Sample(
                t_s, x, y, _wrap_pi(th), v, w, vl, vr,
                xd=xd, yd=yd, thd=_wrap_pi(thd),
                vl_ref=vl_ref, vr_ref=vr_ref, v_ref=v_ref, w_ref=w_ref
            )

        # 5) t,x,y,th,xd,yd,thd,v,w,vl,vr
        if len(parts) == 12:
            t_ms, x, y, th, xd, yd, thd, v, w, vl, vr = map(float, parts[:11])
            t_s = t_ms * 1e-3
            return Sample(t_s, x, y, _wrap_pi(th), v, w, vl, vr, xd=xd, yd=yd, thd=_wrap_pi(thd))

        # 6) t,x,y,th,xd,yd,thd,v,w,vl,vr,vl_ref,vr_ref
        if len(parts) >= 14:
            t_ms, x, y, th, xd, yd, thd, v, w, vl, vr, vl_ref, vr_ref = map(float, parts[:14])
            t_s = t_ms * 1e-3
            v_ref, w_ref = vw_from_wheels(vl_ref, vr_ref)
            return Sample(
                t_s, x, y, _wrap_pi(th), v, w, vl, vr,
                xd=xd, yd=yd, thd=_wrap_pi(thd),
                vl_ref=vl_ref, vr_ref=vr_ref, v_ref=v_ref, w_ref=w_ref
            )
    except Exception:
        return None

    return None


# ----------------------------
# Serial reader thread
# ----------------------------

class SerialWorker(QtCore.QThread):
    sample_rx = QtCore.Signal(object)   # Sample
    status_rx = QtCore.Signal(str)
    stats_rx = QtCore.Signal(int, int)  # lines, parse_errors (since last tick)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ser: Optional[serial.Serial] = None
        self._running = False
        self._paused = False
        self._track_width_in: Optional[float] = None
        self._prefix: str = "P"

        self._lines = 0
        self._parse_err = 0

    def configure(self, *, track_width_in: Optional[float], prefix: str):
        self._track_width_in = track_width_in
        self._prefix = prefix.strip().upper()

    def connect_port(self, port: str, baud: int):
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(1.2)  # allow Arduino reset
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
            self.status_rx.emit(f"Connected: {port} @ {baud}")
        except Exception as e:
            self._ser = None
            self.status_rx.emit(f"Connect failed: {e}")

    def disconnect_port(self):
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self.status_rx.emit("Disconnected")

    def set_paused(self, paused: bool):
        self._paused = paused

    def run(self):
        self._running = True

        # stats timer inside thread
        last_stats = time.monotonic()

        while self._running:
            if not self._ser:
                self.msleep(50)
                continue

            try:
                raw = self._ser.readline()
                if raw:
                    self._lines += 1
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        # Optional prefix filtering: if prefix set and line doesn't match, still allow parse_line
                        if self._prefix:
                            up = line.lstrip().upper()
                            # we don't hard-reject; parse_line will ignore junk prefixes anyway
                            _ = up  # keep for readability

                        s = parse_line(line, track_width_in=self._track_width_in)
                        if s is None:
                            self._parse_err += 1
                        else:
                            if not self._paused:
                                self.sample_rx.emit(s)
            except Exception:
                # transient serial issues
                self.msleep(5)

            now = time.monotonic()
            if now - last_stats >= 1.0:
                self.stats_rx.emit(self._lines, self._parse_err)
                self._lines = 0
                self._parse_err = 0
                last_stats = now

        # cleanup
        self.disconnect_port()

    def stop(self):
        self._running = False


# ----------------------------
# Main UI
# ----------------------------

class RobotDebugUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Serial Debug UI")

        # --- state / buffers ---
        self.max_points = 6000
        self.window_seconds = 12.0  # for time plots (scrolling)
        self.heading_deg = False
        self.arrow_len = 3.0

        # circular buffers
        self.ts: Deque[float] = deque(maxlen=self.max_points)
        self.xs: Deque[float] = deque(maxlen=self.max_points)
        self.ys: Deque[float] = deque(maxlen=self.max_points)
        self.ths: Deque[float] = deque(maxlen=self.max_points)
        self.vs: Deque[float] = deque(maxlen=self.max_points)
        self.ws: Deque[float] = deque(maxlen=self.max_points)
        self.vls: Deque[float] = deque(maxlen=self.max_points)
        self.vrs: Deque[float] = deque(maxlen=self.max_points)

        self.xds: Deque[float] = deque(maxlen=self.max_points)
        self.yds: Deque[float] = deque(maxlen=self.max_points)
        self.thds: Deque[float] = deque(maxlen=self.max_points)

        self.ts_ref: Deque[float] = deque(maxlen=self.max_points)
        self.vs_ref: Deque[float] = deque(maxlen=self.max_points)
        self.ws_ref: Deque[float] = deque(maxlen=self.max_points)
        self.vls_ref: Deque[float] = deque(maxlen=self.max_points)
        self.vrs_ref: Deque[float] = deque(maxlen=self.max_points)

        self._log_fp = None
        self._log_writer = None

        # --- build UI ---
        self._build_controls()
        self._build_plots()
        self._build_status()

        # --- serial worker ---
        self.worker = SerialWorker()
        self.worker.sample_rx.connect(self.on_sample)
        self.worker.status_rx.connect(self.set_status)
        self.worker.stats_rx.connect(self.on_stats)
        self.worker.start()

        # plot update timer (UI thread)
        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self.redraw)
        self.ui_timer.start(33)

        self._rx_lines_per_s = 0
        self._parse_err_per_s = 0
        self._samples_per_s = 0
        self._samples_counter = 0
        self._samples_counter_last = time.monotonic()

        self.refresh_ports()

    # ---------- UI building ----------

    def _build_controls(self):
        w = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(w)

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
        self.track_width.setValue(0.0)
        self.track_width.setSuffix(" in")
        self.track_width.setToolTip("Track width (in). 0 disables ω computation from wheels.")

        self.prefix_edit = QtWidgets.QLineEdit("P")
        self.prefix_edit.setMaximumWidth(60)
        self.prefix_edit.setToolTip("Optional telemetry prefix (e.g., P). Leave empty to accept numeric-first lines.")

        self.heading_unit = QtWidgets.QComboBox()
        self.heading_unit.addItems(["rad", "deg"])

        self.log_btn = QtWidgets.QPushButton("Start Log…")
        self.log_btn.setCheckable(True)

        layout.addWidget(QtWidgets.QLabel("Port:"))
        layout.addWidget(self.port_combo)
        layout.addWidget(self.refresh_btn)
        layout.addSpacing(10)
        layout.addWidget(QtWidgets.QLabel("Baud:"))
        layout.addWidget(self.baud_combo)
        layout.addSpacing(10)
        layout.addWidget(self.connect_btn)
        layout.addWidget(self.disconnect_btn)
        layout.addSpacing(10)
        layout.addWidget(self.pause_btn)
        layout.addWidget(self.clear_btn)
        layout.addSpacing(10)
        layout.addWidget(QtWidgets.QLabel("Track width:"))
        layout.addWidget(self.track_width)
        layout.addWidget(QtWidgets.QLabel("Prefix:"))
        layout.addWidget(self.prefix_edit)
        layout.addWidget(QtWidgets.QLabel("Heading:"))
        layout.addWidget(self.heading_unit)
        layout.addSpacing(10)
        layout.addWidget(self.log_btn)
        layout.addStretch(1)

        self.setCentralWidget(QtWidgets.QWidget())
        root = QtWidgets.QVBoxLayout(self.centralWidget())
        root.addWidget(w)

        # hook up controls
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.pause_btn.toggled.connect(self.on_pause_toggled)
        self.clear_btn.clicked.connect(self.clear_buffers)
        self.heading_unit.currentTextChanged.connect(self.on_heading_unit_changed)
        self.log_btn.toggled.connect(self.on_log_toggled)

    def _build_plots(self):
        container = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(container)
        grid.setContentsMargins(0, 0, 0, 0)

        pg.setConfigOptions(antialias=True)

        # XY plot
        self.xy = pg.PlotWidget(title="World XY trajectory")
        self.xy.setLabel("bottom", "x", units="in")
        self.xy.setLabel("left", "y", units="in")
        self.xy.showGrid(x=True, y=True, alpha=0.2)
        self.xy.setAspectLocked(True, 1)

        self.xy_actual = self.xy.plot([], [], pen=pg.mkPen(width=2), name="actual")
        self.xy_des = self.xy.plot([], [], pen=pg.mkPen(style=QtCore.Qt.DashLine, width=2), name="desired")
        self.xy_actual_pt = pg.ScatterPlotItem(size=8)
        self.xy_des_pt = pg.ScatterPlotItem(size=8, symbol="x")
        self.xy.addItem(self.xy_actual_pt)
        self.xy.addItem(self.xy_des_pt)

        # heading arrow as an item (simple line from pose)
        self.xy_heading = pg.PlotDataItem([], [], pen=pg.mkPen(width=3))
        self.xy.addItem(self.xy_heading)

        # Heading vs time
        self.th_plot = pg.PlotWidget(title="Heading vs time")
        self.th_plot.setLabel("bottom", "t", units="s")
        self.th_plot.setLabel("left", "heading", units="rad")
        self.th_plot.showGrid(x=True, y=True, alpha=0.2)
        self.th_line = self.th_plot.plot([], [], pen=pg.mkPen(width=2))

        # v plot
        self.v_plot = pg.PlotWidget(title="Chassis linear velocity v")
        self.v_plot.setLabel("bottom", "t", units="s")
        self.v_plot.setLabel("left", "v", units="in/s")
        self.v_plot.showGrid(x=True, y=True, alpha=0.2)
        self.v_line = self.v_plot.plot([], [], pen=pg.mkPen(width=2))
        self.v_ref_line = self.v_plot.plot([], [], pen=pg.mkPen(style=QtCore.Qt.DashLine, width=2))

        # w plot
        self.w_plot = pg.PlotWidget(title="Chassis angular velocity ω")
        self.w_plot.setLabel("bottom", "t", units="s")
        self.w_plot.setLabel("left", "ω", units="rad/s")
        self.w_plot.showGrid(x=True, y=True, alpha=0.2)
        self.w_line = self.w_plot.plot([], [], pen=pg.mkPen(width=2))
        self.w_ref_line = self.w_plot.plot([], [], pen=pg.mkPen(style=QtCore.Qt.DashLine, width=2))

        # wheels plot
        self.wh_plot = pg.PlotWidget(title="Wheel velocities")
        self.wh_plot.setLabel("bottom", "t", units="s")
        self.wh_plot.setLabel("left", "wheel v", units="in/s")
        self.wh_plot.showGrid(x=True, y=True, alpha=0.2)
        self.vl_line = self.wh_plot.plot([], [], pen=pg.mkPen(width=2))
        self.vr_line = self.wh_plot.plot([], [], pen=pg.mkPen(width=2))
        self.vl_ref_line = self.wh_plot.plot([], [], pen=pg.mkPen(style=QtCore.Qt.DashLine, width=2))
        self.vr_ref_line = self.wh_plot.plot([], [], pen=pg.mkPen(style=QtCore.Qt.DashLine, width=2))

        # layout: 2 columns, XY bigger
        grid.addWidget(self.xy, 0, 0, 2, 1)
        grid.addWidget(self.th_plot, 0, 1)
        grid.addWidget(self.v_plot, 1, 1)
        grid.addWidget(self.w_plot, 2, 1)
        grid.addWidget(self.wh_plot, 2, 0)

        # stretch factors
        grid.setRowStretch(0, 2)
        grid.setRowStretch(1, 1)
        grid.setRowStretch(2, 1)
        grid.setColumnStretch(0, 2)
        grid.setColumnStretch(1, 1)

        self.centralWidget().layout().addWidget(container)

    def _build_status(self):
        self.status = QtWidgets.QStatusBar()
        self.setStatusBar(self.status)
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.stats_label = QtWidgets.QLabel("RX: 0 l/s | samples: 0 sps | parse err: 0 l/s")
        self.status.addWidget(self.status_label, 1)
        self.status.addPermanentWidget(self.stats_label)

    # ---------- helpers ----------

    def set_status(self, msg: str):
        self.status_label.setText(msg)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = list(list_ports.comports())
        # prefer Arduino-ish ports by description/manufacturer
        def score(p):
            desc = (p.description or "").lower()
            manu = (p.manufacturer or "").lower()
            s = 0
            if "arduino" in desc or "arduino" in manu:
                s += 10
            if "usb" in desc and ("serial" in desc or "cdc" in desc):
                s += 5
            return -s  # for ascending sort

        ports_sorted = sorted(ports, key=score)
        for p in ports_sorted:
            label = f"{p.device} — {p.description}"
            self.port_combo.addItem(label, p.device)

        if self.port_combo.count() == 0:
            self.port_combo.addItem("(no ports found)", "")

    def current_port(self) -> str:
        return self.port_combo.currentData() or ""

    def current_baud(self) -> int:
        return int(self.baud_combo.currentData())

    def clear_buffers(self):
        for dq in (
            self.ts, self.xs, self.ys, self.ths, self.vs, self.ws, self.vls, self.vrs,
            self.xds, self.yds, self.thds,
            self.ts_ref, self.vs_ref, self.ws_ref, self.vls_ref, self.vrs_ref
        ):
            dq.clear()

    # ---------- control callbacks ----------

    def on_connect(self):
        port = self.current_port()
        if not port:
            self.set_status("No port selected")
            return
        baud = self.current_baud()

        tw = float(self.track_width.value())
        track_width_in = tw if tw > 0 else None
        prefix = self.prefix_edit.text()

        self.worker.configure(track_width_in=track_width_in, prefix=prefix)
        self.worker.connect_port(port, baud)

        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)

    def on_disconnect(self):
        self.worker.disconnect_port()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)

    def on_pause_toggled(self, paused: bool):
        self.worker.set_paused(paused)
        self.pause_btn.setText("Resume" if paused else "Pause")

    def on_heading_unit_changed(self, unit: str):
        self.heading_deg = (unit.lower() == "deg")
        self.th_plot.setLabel("left", "heading", units=("deg" if self.heading_deg else "rad"))

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
                    "t_s", "x_in", "y_in", "th_rad", "v_in_s", "w_rad_s", "vl_in_s", "vr_in_s",
                    "xd_in", "yd_in", "thd_rad",
                    "vl_ref_in_s", "vr_ref_in_s", "v_ref_in_s", "w_ref_rad_s"
                ])
                self.log_btn.setText("Stop Log")
                self.set_status(f"Logging to {path}")
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
            self.set_status("Logging stopped")

    # ---------- data ingest ----------

    @QtCore.Slot(object)
    def on_sample(self, s: Sample):
        self.ts.append(s.t)
        self.xs.append(s.x)
        self.ys.append(s.y)
        self.ths.append(_wrap_pi(s.th))
        self.vs.append(s.v)
        self.ws.append(s.w)
        self.vls.append(s.vl)
        self.vrs.append(s.vr)

        if s.xd is not None and s.yd is not None and s.thd is not None:
            self.xds.append(s.xd)
            self.yds.append(s.yd)
            self.thds.append(_wrap_pi(s.thd))

        # refs (keep separate time base)
        if s.v_ref is not None:
            self.ts_ref.append(s.t)
            self.vs_ref.append(s.v_ref)
        if s.w_ref is not None:
            # align with ts_ref if needed
            if len(self.ts_ref) == 0 or self.ts_ref[-1] != s.t:
                self.ts_ref.append(s.t)
            self.ws_ref.append(s.w_ref)
        if s.vl_ref is not None:
            if len(self.ts_ref) == 0 or self.ts_ref[-1] != s.t:
                self.ts_ref.append(s.t)
            self.vls_ref.append(s.vl_ref)
        if s.vr_ref is not None:
            if len(self.ts_ref) == 0 or self.ts_ref[-1] != s.t:
                self.ts_ref.append(s.t)
            self.vrs_ref.append(s.vr_ref)

        # logging
        if self._log_writer:
            self._log_writer.writerow([
                s.t, s.x, s.y, s.th, s.v, s.w, s.vl, s.vr,
                s.xd if s.xd is not None else "",
                s.yd if s.yd is not None else "",
                s.thd if s.thd is not None else "",
                s.vl_ref if s.vl_ref is not None else "",
                s.vr_ref if s.vr_ref is not None else "",
                s.v_ref if s.v_ref is not None else "",
                s.w_ref if s.w_ref is not None else "",
            ])

        # samples/sec
        self._samples_counter += 1
        now = time.monotonic()
        if now - self._samples_counter_last >= 1.0:
            self._samples_per_s = self._samples_counter
            self._samples_counter = 0
            self._samples_counter_last = now

    @QtCore.Slot(int, int)
    def on_stats(self, lines: int, parse_err: int):
        self._rx_lines_per_s = lines
        self._parse_err_per_s = parse_err
        self.stats_label.setText(
            f"RX: {self._rx_lines_per_s} l/s | samples: {self._samples_per_s} sps | parse err: {self._parse_err_per_s} l/s"
        )

    # ---------- plotting ----------

    def redraw(self):
        if len(self.ts) < 2:
            return

        # convert to numpy for slicing/plotting
        ts = np.asarray(self.ts, dtype=float)

        # show only last window_seconds in time plots
        t_now = ts[-1]
        t_min = t_now - self.window_seconds
        idx0 = int(np.searchsorted(ts, t_min, side="left"))
        tsw = ts[idx0:]

        def arr(dq: Deque[float]) -> np.ndarray:
            a = np.asarray(dq, dtype=float)
            return a[idx0:] if len(a) == len(ts) else a  # time-aligned buffers only

        xs = np.asarray(self.xs, dtype=float)
        ys = np.asarray(self.ys, dtype=float)

        # XY (full buffer, not windowed)
        self.xy_actual.setData(xs, ys)
        self.xy_actual_pt.setData([{"pos": (xs[-1], ys[-1])}])

        if len(self.xds) > 0 and len(self.yds) > 0:
            xd = np.asarray(self.xds, dtype=float)
            yd = np.asarray(self.yds, dtype=float)
            self.xy_des.setData(xd, yd)
            self.xy_des_pt.setData([{"pos": (xd[-1], yd[-1])}])
        else:
            self.xy_des.setData([], [])
            self.xy_des_pt.setData([])

        # heading arrow line
        th = float(self.ths[-1])
        x0, y0 = float(xs[-1]), float(ys[-1])
        x1 = x0 + self.arrow_len * math.cos(th)
        y1 = y0 + self.arrow_len * math.sin(th)
        self.xy_heading.setData([x0, x1], [y0, y1])

        # heading plot
        ths = arr(self.ths)
        if self.heading_deg:
            ths_plot = np.degrees(ths)
        else:
            ths_plot = ths
        self.th_line.setData(tsw, ths_plot)
        self.th_plot.setXRange(t_min, t_now, padding=0.0)

        # v/w plots
        self.v_line.setData(tsw, arr(self.vs))
        self.w_line.setData(tsw, arr(self.ws))
        self.v_plot.setXRange(t_min, t_now, padding=0.0)
        self.w_plot.setXRange(t_min, t_now, padding=0.0)

        # wheels
        self.vl_line.setData(tsw, arr(self.vls))
        self.vr_line.setData(tsw, arr(self.vrs))
        self.wh_plot.setXRange(t_min, t_now, padding=0.0)

        # refs (only plot if present)
        if len(self.ts_ref) > 2:
            tsr = np.asarray(self.ts_ref, dtype=float)
            tminr = t_now - self.window_seconds
            j0 = int(np.searchsorted(tsr, tminr, side="left"))
            tsr_w = tsr[j0:]

            def arr_ref(dq: Deque[float]) -> np.ndarray:
                a = np.asarray(dq, dtype=float)
                # dq lengths may differ; best-effort window
                return a[j0:] if len(a) == len(tsr) else a[-len(tsr_w):]

            if len(self.vs_ref) > 0:
                self.v_ref_line.setData(tsr_w, arr_ref(self.vs_ref))
            else:
                self.v_ref_line.setData([], [])

            if len(self.ws_ref) > 0:
                self.w_ref_line.setData(tsr_w, arr_ref(self.ws_ref))
            else:
                self.w_ref_line.setData([], [])

            if len(self.vls_ref) > 0:
                self.vl_ref_line.setData(tsr_w, arr_ref(self.vls_ref))
            else:
                self.vl_ref_line.setData([], [])

            if len(self.vrs_ref) > 0:
                self.vr_ref_line.setData(tsr_w, arr_ref(self.vrs_ref))
            else:
                self.vr_ref_line.setData([], [])

    # ---------- shutdown ----------

    def closeEvent(self, event):
        try:
            self.worker.stop()
            self.worker.wait(800)
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
    win = RobotDebugUI()
    win.resize(1300, 850)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())