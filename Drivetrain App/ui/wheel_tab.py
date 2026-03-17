
from __future__ import annotations

from PySide6 import QtCore, QtWidgets

from plotting import MplCanvas, canvas_with_toolbar, style_axes


class WheelTab(QtWidgets.QWidget):
    wheel_popout_requested = QtCore.Signal()

    def __init__(self, line_width: float, parent=None):
        super().__init__(parent)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        self.wheel_canvas = MplCanvas()
        gs = self.wheel_canvas.fig.add_gridspec(2, 2, height_ratios=[1.0, 1.0])

        self.ax_wheel_vel = self.wheel_canvas.fig.add_subplot(gs[0, :])
        self.ax_left_pos = self.wheel_canvas.fig.add_subplot(gs[1, 0])
        self.ax_right_pos = self.wheel_canvas.fig.add_subplot(gs[1, 1])

        style_axes(self.ax_wheel_vel, "Wheel velocities", "sample", "wheel v [in/s]")
        style_axes(self.ax_left_pos, "Left wheel position", "sample", "position")
        style_axes(self.ax_right_pos, "Right wheel position", "sample", "position")

        (self.vl_line,) = self.ax_wheel_vel.plot([], [], linewidth=line_width, label="left actual")
        (self.vl_ref_line,) = self.ax_wheel_vel.plot([], [], "--", linewidth=line_width, label="left desired")
        (self.vr_line,) = self.ax_wheel_vel.plot([], [], linewidth=line_width, label="right actual")
        (self.vr_ref_line,) = self.ax_wheel_vel.plot([], [], "--", linewidth=line_width, label="right desired")
        self.ax_wheel_vel.legend(loc="upper left")

        (self.left_pos_line,) = self.ax_left_pos.plot([], [], linewidth=line_width, label="left actual")
        (self.left_pos_ref_line,) = self.ax_left_pos.plot([], [], "--", linewidth=line_width, label="left desired")
        self.ax_left_pos.legend(loc="upper left")

        (self.right_pos_line,) = self.ax_right_pos.plot([], [], linewidth=line_width, label="right actual")
        (self.right_pos_ref_line,) = self.ax_right_pos.plot([], [], "--", linewidth=line_width, label="right desired")
        self.ax_right_pos.legend(loc="upper left")

        self.wheel_canvas.double_clicked.connect(self.wheel_popout_requested.emit)
        outer.addWidget(canvas_with_toolbar(self.wheel_canvas), 1)