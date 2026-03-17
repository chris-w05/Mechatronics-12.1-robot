
from __future__ import annotations

import sys

from PySide6 import QtWidgets

from ui.main_window import RobotDebugUI


def main():
    app = QtWidgets.QApplication(sys.argv)

    app.setStyleSheet(
        """
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
        """
    )

    win = RobotDebugUI()
    win.resize(1550, 980)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())