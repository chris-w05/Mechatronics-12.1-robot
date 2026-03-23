#!/usr/bin/env python3
"""
main.py  –  Entry point for the Robot Simulator

Usage:
    python -m Simulator
    # or
    python Simulator/main.py
"""

import sys
from PySide6 import QtWidgets
from Simulator.gui import SimulatorWindow, STYLESHEET


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(STYLESHEET)
    window = SimulatorWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
