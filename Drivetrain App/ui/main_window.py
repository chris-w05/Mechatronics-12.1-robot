
from __future__ import annotations

import csv
import math
import time
from collections import deque
from typing import Deque

import numpy as np
from PySide6 import QtCore, QtWidgets
from serial.tools import list_ports

from mpl_theme import setup_dark_mpl
from parser import wrap_pi
from plotting import PlotContainer, PopoutWindow, MplCanvas, autoscale_y, last_window, set_scroll_xlim
from serial_worker import SerialWorker
from ui.console_tab import ConsoleTab
from ui.controls import ControlsPanel
from ui.pose_tab import PoseTab
from ui.wheel_tab import WheelTab


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
        self._popouts = []
        self._canvas_homes = {}
        self._canvas_popouts = {}

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

    def _build_ui(self):
        self.setCentralWidget(QtWidgets.QWidget())
        root = QtWidgets.QVBoxLayout(self.centralWidget())

        self.controls = ControlsPanel(self.max_history, self.window_points)
        root.addWidget(self.controls)

        self.tabs = QtWidgets.QTabWidget()
        self.pose_tab = PoseTab(self.line_width, self.heading_line_width)
        self.wheel_tab = WheelTab(self.line_width)
        self.console_tab = ConsoleTab(self.console_max_lines)

        self.tabs.addTab(self.pose_tab, "Pose")
        self.tabs.addTab(self.wheel_tab, "Wheels")
        self.tabs.addTab(self.console_tab, "Serial Console")
        root.addWidget(self.tabs, 1)

        self._build_status()
        self._wire_signals()
        self._remember_canvas_homes()

    def _remember_canvas_homes(self):
        self._canvas_homes = {
            self.pose_tab.pose_canvas: self.pose_tab.pose_canvas.parentWidget(),
            self.pose_tab.angvel_canvas: self.pose_tab.angvel_canvas.parentWidget(),
            self.wheel_tab.wheel_canvas: self.wheel_tab.wheel_canvas.parentWidget(),
        }

    def _wire_signals(self):
        c = self.controls
        con = self.console_tab
        c.refresh_btn.clicked.connect(self.refresh_ports)
        c.connect_btn.clicked.connect(self.on_connect)
        c.disconnect_btn.clicked.connect(self.on_disconnect)
        c.pause_btn.toggled.connect(self.on_pause_toggled)
        c.clear_btn.clicked.connect(self.clear_buffers)
        c.heading_unit.currentTextChanged.connect(self.on_heading_unit_changed)
        c.log_btn.toggled.connect(self.on_log_toggled)
        c.window_spin.valueChanged.connect(self.on_window_changed)

        con.send_btn.clicked.connect(self.on_send_command)
        con.console_input.returnPressed.connect(self.on_send_command)
        con.clear_console_btn.clicked.connect(con.console_view.clear)

        self.pose_tab.pose_popout_requested.connect(lambda: self._pop_out_canvas(self.pose_tab.pose_canvas, "Pose overview"))
        self.pose_tab.angvel_popout_requested.connect(lambda: self._pop_out_canvas(self.pose_tab.angvel_canvas, "Angular velocity"))
        self.wheel_tab.wheel_popout_requested.connect(lambda: self._pop_out_canvas(self.wheel_tab.wheel_canvas, "Wheels"))

    def _pop_out_canvas(self, canvas: MplCanvas, title: str):
        existing = self._canvas_popouts.get(canvas)
        if existing is not None:
            existing.raise_()
            existing.activateWindow()
            return

        home = self._canvas_homes.get(canvas)
        if not isinstance(home, PlotContainer):
            return

        home_layout = home.layout()
        if home_layout is None:
            return

        home_layout.removeWidget(canvas)
        canvas.setParent(None)

        win = PopoutWindow(title, canvas, self)

        def restore_canvas():
            if self._canvas_popouts.get(canvas) is not win:
                return
            self._canvas_popouts.pop(canvas, None)
            layout = home.layout()
            if layout is not None:
                layout.addWidget(canvas, 1)
            canvas.draw_idle()

        win.closed.connect(restore_canvas)
        win.destroyed.connect(lambda *_: self._canvas_popouts.pop(canvas, None))
        win.show()
        canvas.draw_idle()
        self._canvas_popouts[canvas] = win
        self._popouts.append(win)

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
        c = self.controls
        c.port_combo.clear()
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
            c.port_combo.addItem(f"{p.device} — {p.description}", p.device)
        if c.port_combo.count() == 0:
            c.port_combo.addItem("(no ports found)", "")

    def current_port(self) -> str:
        return self.controls.port_combo.currentData() or ""

    def current_baud(self) -> int:
        return int(self.controls.baud_combo.currentData())

    def on_window_changed(self, value: int):
        self.window_points = int(value)
        self._dirty = True

    def _iter_lines(self):
        p = self.pose_tab
        w = self.wheel_tab
        return [
            p.xy_actual_line, p.xy_des_line, p.xy_actual_pt, p.xy_des_pt,
            p.xy_heading_line, p.xy_heading_des_line, p.th_line,
            p.v_line, p.v_ref_line, p.w_line, p.w_ref_line,
            p.ex_line, p.ey_line, p.eth_line,
            w.vl_line, w.vl_ref_line, w.vr_line, w.vr_ref_line,
            w.left_pos_line, w.left_pos_ref_line, w.right_pos_line, w.right_pos_ref_line,
        ]

    def _iter_axes(self):
        p = self.pose_tab
        w = self.wheel_tab
        return [
            p.ax_xy, p.ax_th, p.ax_v, p.ax_w, p.ax_err,
            w.ax_wheel_vel, w.ax_left_pos, w.ax_right_pos,
        ]

    def _clear_plot_lines(self):
        for line in self._iter_lines():
            line.set_data([], [])

        for ax in self._iter_axes():
            ax.relim()
            ax.autoscale_view()

        self.pose_tab.pose_canvas.draw_idle()
        self.pose_tab.angvel_canvas.draw_idle()
        self.wheel_tab.wheel_canvas.draw_idle()

    def clear_buffers(self):
        for dq in [
            self.sample_idx, self.ts, self.xs, self.ys, self.ths, self.vs, self.ws, self.vls, self.vrs, self.pls, self.prs,
            self.xds, self.yds, self.thds, self.vs_ref, self.ws_ref, self.vls_ref, self.vrs_ref, self.pls_ref, self.prs_ref,
            self.exs, self.eys, self.eths,
        ]:
            dq.clear()

        self._next_index = 0
        self.console_tab.console_view.clear()
        self._clear_plot_lines()

    def on_connect(self):
        port = self.current_port()
        if not port:
            self.set_status("No port selected")
            return

        c = self.controls
        QtCore.QMetaObject.invokeMethod(
            self.worker,
            "configure",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(float, float(c.track_width.value())),
            QtCore.Q_ARG(str, c.prefix_edit.text()),
        )

        QtCore.QMetaObject.invokeMethod(
            self.worker,
            "connect_port",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(str, port),
            QtCore.Q_ARG(int, self.current_baud()),
        )

        c.connect_btn.setEnabled(False)
        c.disconnect_btn.setEnabled(True)

    def on_disconnect(self):
        QtCore.QMetaObject.invokeMethod(self.worker, "disconnect_port", QtCore.Qt.QueuedConnection)
        self.controls.connect_btn.setEnabled(True)
        self.controls.disconnect_btn.setEnabled(False)

    def on_pause_toggled(self, paused: bool):
        QtCore.QMetaObject.invokeMethod(
            self.worker,
            "set_paused",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(bool, paused),
        )
        self.controls.pause_btn.setText("Resume" if paused else "Pause")

    def on_heading_unit_changed(self, unit: str):
        p = self.pose_tab
        self.heading_deg = unit.lower() == "deg"
        p.ax_th.set_ylabel("heading [deg]" if self.heading_deg else "heading [rad]")
        p.ax_err.set_ylabel("error (e_th in deg)" if self.heading_deg else "error")
        self._dirty = True

    def on_log_toggled(self, enabled: bool):
        c = self.controls
        if enabled:
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save CSV Log", "robot_telemetry.csv", "CSV Files (*.csv)"
            )
            if not path:
                c.log_btn.setChecked(False)
                return
            try:
                self._log_fp = open(path, "w", newline="", buffering=1)
                self._log_writer = csv.writer(self._log_fp)
                self._log_writer.writerow([
                    "sample", "t_s", "x", "y", "th", "v", "w", "vl", "vr", "pl", "pr",
                    "xd", "yd", "thd", "vl_ref", "vr_ref", "v_ref", "w_ref", "pl_ref", "pr_ref",
                ])
                c.log_btn.setText("Stop Log")
            except Exception as e:
                self.set_status(f"Log open failed: {e}")
                c.log_btn.setChecked(False)
        else:
            c.log_btn.setText("Start Log…")
            try:
                if self._log_fp:
                    self._log_fp.close()
            except Exception:
                pass
            self._log_fp = None
            self._log_writer = None

    def on_send_command(self):
        con = self.console_tab
        text = con.console_input.text().strip()
        if not text:
            return
        con.console_view.appendPlainText(f"> {text}")
        QtCore.QMetaObject.invokeMethod(
            self.worker,
            "send_line",
            QtCore.Qt.QueuedConnection,
            QtCore.Q_ARG(str, text),
        )
        con.console_input.clear()

    @QtCore.Slot(str)
    def on_raw_line(self, line: str):
        self.console_tab.console_view.appendPlainText(line)

    @QtCore.Slot(object)
    def on_sample(self, s):
        idx = self._next_index
        self._next_index += 1

        self.sample_idx.append(idx)
        self.ts.append(s.t)
        self.xs.append(s.x)
        self.ys.append(s.y)
        self.ths.append(wrap_pi(s.th))
        self.vs.append(s.v)
        self.ws.append(s.w)
        self.vls.append(s.vl)
        self.vrs.append(s.vr)
        self.pls.append(np.nan if s.pl is None else s.pl)
        self.prs.append(np.nan if s.pr is None else s.pr)

        self.xds.append(np.nan if s.xd is None else s.xd)
        self.yds.append(np.nan if s.yd is None else s.yd)
        self.thds.append(np.nan if s.thd is None else wrap_pi(s.thd))
        self.vs_ref.append(np.nan if s.v_ref is None else s.v_ref)
        self.ws_ref.append(np.nan if s.w_ref is None else s.w_ref)
        self.vls_ref.append(np.nan if s.vl_ref is None else s.vl_ref)
        self.vrs_ref.append(np.nan if s.vr_ref is None else s.vr_ref)
        self.pls_ref.append(np.nan if s.pl_ref is None else s.pl_ref)
        self.prs_ref.append(np.nan if s.pr_ref is None else s.pr_ref)

        if s.xd is not None and s.yd is not None and s.thd is not None:
            self.exs.append(s.xd - s.x)
            self.eys.append(s.yd - s.y)
            self.eths.append(wrap_pi(s.thd - s.th))
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

    def redraw(self):
        if not self._dirty or len(self.sample_idx) < 1:
            return

        p = self.pose_tab
        w = self.wheel_tab

        idx = last_window(self.sample_idx, self.window_points)
        xs = last_window(self.xs, self.window_points)
        ys = last_window(self.ys, self.window_points)
        ths = last_window(self.ths, self.window_points)
        vs = last_window(self.vs, self.window_points)
        ws = last_window(self.ws, self.window_points)
        vls = last_window(self.vls, self.window_points)
        vrs = last_window(self.vrs, self.window_points)
        pls = last_window(self.pls, self.window_points)
        prs = last_window(self.prs, self.window_points)

        xds = last_window(self.xds, self.window_points)
        yds = last_window(self.yds, self.window_points)
        thds = last_window(self.thds, self.window_points)
        vs_ref = last_window(self.vs_ref, self.window_points)
        ws_ref = last_window(self.ws_ref, self.window_points)
        vls_ref = last_window(self.vls_ref, self.window_points)
        vrs_ref = last_window(self.vrs_ref, self.window_points)
        pls_ref = last_window(self.pls_ref, self.window_points)
        prs_ref = last_window(self.prs_ref, self.window_points)
        exs = last_window(self.exs, self.window_points)
        eys = last_window(self.eys, self.window_points)
        eths = last_window(self.eths, self.window_points)

        xs_full = np.asarray(self.xs, dtype=float)
        ys_full = np.asarray(self.ys, dtype=float)
        xds_full = np.asarray(self.xds, dtype=float)
        yds_full = np.asarray(self.yds, dtype=float)
        thds_full = np.asarray(self.thds, dtype=float)

        p.xy_actual_line.set_data(xs_full, ys_full)
        if xs.size and ys.size:
            p.xy_actual_pt.set_data([xs[-1]], [ys[-1]])
            th = float(ths[-1])
            x0, y0 = float(xs[-1]), float(ys[-1])
            p.xy_heading_line.set_data(
                [x0, x0 + self.arrow_len * math.cos(th)],
                [y0, y0 + self.arrow_len * math.sin(th)],
            )
        else:
            p.xy_actual_pt.set_data([], [])
            p.xy_heading_line.set_data([], [])

        valid_des_full = np.isfinite(xds_full) & np.isfinite(yds_full)
        p.xy_des_line.set_data(xds_full[valid_des_full], yds_full[valid_des_full])
        if np.any(valid_des_full):
            xd_last = xds_full[valid_des_full][-1]
            yd_last = yds_full[valid_des_full][-1]
            p.xy_des_pt.set_data([xd_last], [yd_last])

            valid_thd_full = np.isfinite(thds_full)
            if np.any(valid_thd_full):
                thd_last = thds_full[valid_thd_full][-1]
                p.xy_heading_des_line.set_data(
                    [xd_last, xd_last + self.arrow_len * math.cos(thd_last)],
                    [yd_last, yd_last + self.arrow_len * math.sin(thd_last)],
                )
            else:
                p.xy_heading_des_line.set_data([], [])
        else:
            p.xy_des_pt.set_data([], [])
            p.xy_heading_des_line.set_data([], [])

        th_plot = np.degrees(ths) if self.heading_deg else ths
        p.th_line.set_data(idx, th_plot)
        set_scroll_xlim(p.ax_th, idx)
        autoscale_y(p.ax_th, [th_plot])

        p.v_line.set_data(idx, vs)
        p.v_ref_line.set_data(idx, vs_ref)
        set_scroll_xlim(p.ax_v, idx)
        autoscale_y(p.ax_v, [vs, vs_ref])

        p.w_line.set_data(idx, ws)
        p.w_ref_line.set_data(idx, ws_ref)
        set_scroll_xlim(p.ax_w, idx)
        autoscale_y(p.ax_w, [ws, ws_ref])

        eth_plot = np.degrees(eths) if self.heading_deg else eths
        p.ex_line.set_data(idx, exs)
        p.ey_line.set_data(idx, eys)
        p.eth_line.set_data(idx, eth_plot)
        set_scroll_xlim(p.ax_err, idx)
        autoscale_y(p.ax_err, [exs, eys, eth_plot])

        w.vl_line.set_data(idx, vls)
        w.vl_ref_line.set_data(idx, vls_ref)
        w.vr_line.set_data(idx, vrs)
        w.vr_ref_line.set_data(idx, vrs_ref)
        set_scroll_xlim(w.ax_wheel_vel, idx)
        autoscale_y(w.ax_wheel_vel, [vls, vls_ref, vrs, vrs_ref])

        w.left_pos_line.set_data(idx, pls)
        w.left_pos_ref_line.set_data(idx, pls_ref)
        set_scroll_xlim(w.ax_left_pos, idx)
        autoscale_y(w.ax_left_pos, [pls, pls_ref])

        w.right_pos_line.set_data(idx, prs)
        w.right_pos_ref_line.set_data(idx, prs_ref)
        set_scroll_xlim(w.ax_right_pos, idx)
        autoscale_y(w.ax_right_pos, [prs, prs_ref])

        p.pose_canvas.draw_idle()
        p.angvel_canvas.draw_idle()
        w.wheel_canvas.draw_idle()
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