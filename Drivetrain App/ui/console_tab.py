
from __future__ import annotations

from PySide6 import QtWidgets


class ConsoleTab(QtWidgets.QWidget):
    def __init__(self, console_max_lines: int, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)

        self.console_view = QtWidgets.QPlainTextEdit()
        self.console_view.setReadOnly(True)
        self.console_view.setMaximumBlockCount(console_max_lines)

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