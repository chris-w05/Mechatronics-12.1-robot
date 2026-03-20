
from __future__ import annotations

from PySide6 import QtCore, QtWidgets

from plotting import MplCanvas, canvas_with_toolbar, style_axes


class PoseTab(QtWidgets.QWidget):
    pose_popout_requested = QtCore.Signal()
    angvel_popout_requested = QtCore.Signal()

    def __init__(self, line_width: float, heading_line_width: float, parent=None):
        super().__init__(parent)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        self.pose_canvas = MplCanvas()
        gs = self.pose_canvas.fig.add_gridspec(
            3, 2,
            width_ratios=[1.7, 1.0],
            height_ratios=[1.0, 1.0, 0.48],
        )

        self.ax_xy = self.pose_canvas.fig.add_subplot(gs[:, 0])
        self.ax_th = self.pose_canvas.fig.add_subplot(gs[0, 1])
        self.ax_v = self.pose_canvas.fig.add_subplot(gs[1, 1])
        self.ax_err = self.pose_canvas.fig.add_subplot(gs[2, 1])

        style_axes(self.ax_xy, "World XY trajectory", "x [in]", "y [in]")
        self.ax_xy.set_xlim(0, 96)
        self.ax_xy.set_ylim(0, 48)
        self.ax_xy.set_aspect('equal', adjustable='box')
        style_axes(self.ax_th, "Heading", "sample", "heading [rad]")
        style_axes(self.ax_v, "Chassis linear velocity", "sample", "v [in/s]")
        style_axes(self.ax_err, "Pose error", "sample", "error")

        (self.xy_actual_line,) = self.ax_xy.plot([], [], linewidth=line_width, label="actual path")
        (self.xy_des_line,) = self.ax_xy.plot([], [], "--", linewidth=line_width, label="desired path")
        (self.xy_actual_pt,) = self.ax_xy.plot([], [], "o", markersize=3.0, label="_nolegend_")
        (self.xy_des_pt,) = self.ax_xy.plot([], [], "x", markersize=4.0, mew=0.8, label="_nolegend_")
        (self.xy_heading_line,) = self.ax_xy.plot([], [], linewidth=heading_line_width, label="actual heading")
        (self.xy_heading_des_line,) = self.ax_xy.plot([], [], "--", linewidth=heading_line_width, label="desired heading")
        self.ax_xy.legend(loc="upper left")

        (self.th_line,) = self.ax_th.plot([], [], linewidth=line_width, label="heading")
        self.ax_th.legend(loc="upper left")

        (self.v_line,) = self.ax_v.plot([], [], linewidth=line_width, label="actual")
        (self.v_ref_line,) = self.ax_v.plot([], [], "--", linewidth=line_width, label="desired")
        self.ax_v.legend(loc="upper left")

        (self.ex_line,) = self.ax_err.plot([], [], linewidth=line_width, label="x error")
        (self.ey_line,) = self.ax_err.plot([], [], linewidth=line_width, label="y error")
        (self.eth_line,) = self.ax_err.plot([], [], "--", linewidth=line_width, label="heading error")
        self.ax_err.legend(loc="upper left")

        self.angvel_canvas = MplCanvas()
        self.ax_w = self.angvel_canvas.fig.add_subplot(111)
        style_axes(self.ax_w, "Chassis angular velocity", "sample", "ω [rad/s]")
        (self.w_line,) = self.ax_w.plot([], [], linewidth=line_width, label="actual")
        (self.w_ref_line,) = self.ax_w.plot([], [], "--", linewidth=line_width, label="desired")
        self.ax_w.legend(loc="upper left")

        self.pose_canvas.double_clicked.connect(self.pose_popout_requested.emit)
        self.angvel_canvas.double_clicked.connect(self.angvel_popout_requested.emit)

        split = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        split.addWidget(canvas_with_toolbar(self.pose_canvas))
        split.addWidget(canvas_with_toolbar(self.angvel_canvas))
        split.setStretchFactor(0, 5)
        split.setStretchFactor(1, 2)

        outer.addWidget(split, 1)