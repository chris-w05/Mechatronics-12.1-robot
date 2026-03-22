
from __future__ import annotations

from PySide6 import QtWidgets


class ControlsPanel(QtWidgets.QWidget):
    def __init__(self, max_history: int, window_points: int, parent=None):
        super().__init__(parent)

        outer = QtWidgets.QVBoxLayout(self)
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
        self.track_width.setValue(9.3700)
        self.track_width.setSuffix(" in")

        self.window_spin = QtWidgets.QSpinBox()
        self.window_spin.setRange(20, max_history)
        self.window_spin.setSingleStep(100)
        self.window_spin.setValue(window_points)

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