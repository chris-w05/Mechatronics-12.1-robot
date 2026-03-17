
from __future__ import annotations

import math
from typing import Deque

import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from PySide6 import QtCore, QtWidgets


class MplCanvas(FigureCanvas):
    double_clicked = QtCore.Signal()

    def __init__(self, parent=None):
        self.fig = Figure(constrained_layout=False)
        super().__init__(self.fig)
        self.setParent(parent)
        self.fig.subplots_adjust(left=0.07, right=0.995, top=0.96, bottom=0.10, wspace=0.22, hspace=0.28)

    def mouseDoubleClickEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.double_clicked.emit()
        super().mouseDoubleClickEvent(event)


class CompactNavigationToolbar(NavigationToolbar):
    toolitems = [
        t for t in NavigationToolbar.toolitems
        if t[0] in ("Home", "Back", "Forward", "Pan", "Zoom", "Save")
    ]

    def __init__(self, canvas, parent):
        super().__init__(canvas, parent)
        self.setIconSize(QtCore.QSize(12, 12))
        self.setMaximumHeight(24)
        self.setMovable(False)
        layout = self.layout()
        if layout is not None:
            layout.setContentsMargins(2, 1, 2, 1)
            layout.setSpacing(1)
        self.setStyleSheet(
            """
            QToolBar {
                background-color: #101010;
                border: none;
                spacing: 1px;
                padding: 0px;
            }
            QToolButton {
                background-color: #181818;
                color: #dddddd;
                border: 1px solid #2c2c2c;
                border-radius: 2px;
                padding: 1px;
                margin: 0px;
                min-width: 18px;
                min-height: 18px;
                max-width: 20px;
                max-height: 20px;
            }
            QToolButton:hover {
                background-color: #222222;
            }
            QLabel {
                color: #9a9a9a;
                font-size: 9px;
                padding: 0 2px;
            }
            """
        )
        for action in self.actions():
            widget = self.widgetForAction(action)
            if isinstance(widget, QtWidgets.QToolButton):
                widget.setToolButtonStyle(QtCore.Qt.ToolButtonIconOnly)
                widget.setAutoRaise(True)


class PlotContainer(QtWidgets.QWidget):
    def __init__(self, canvas: MplCanvas, parent=None):
        super().__init__(parent)
        self.canvas = canvas
        self.toolbar = CompactNavigationToolbar(canvas, self)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(1)

        top_row = QtWidgets.QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)
        top_row.setSpacing(4)

        hint = QtWidgets.QLabel("Double-click to pop out")
        hint.setStyleSheet("color: #8a8a8a; font-size: 9px;")
        hint.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        top_row.addWidget(self.toolbar, 0)
        top_row.addStretch(1)
        top_row.addWidget(hint, 0)

        layout.addLayout(top_row)
        layout.addWidget(canvas, 1)


def canvas_with_toolbar(canvas: MplCanvas) -> PlotContainer:
    return PlotContainer(canvas)


class PopoutWindow(QtWidgets.QMainWindow):
    closed = QtCore.Signal()

    def __init__(self, title: str, canvas: MplCanvas, parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(950, 700)
        self._canvas = canvas

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(2)
        layout.addWidget(CompactNavigationToolbar(canvas, central), 0)
        layout.addWidget(canvas, 1)
        self.setCentralWidget(central)

    def closeEvent(self, event):
        self.closed.emit()
        super().closeEvent(event)


def style_axes(ax, title: str, xlabel: str, ylabel: str) -> None:
    ax.set_title(title, pad=2)
    ax.set_xlabel(xlabel, labelpad=1.5)
    ax.set_ylabel(ylabel, labelpad=1.5)
    ax.grid(True, linewidth=0.45)
    ax.tick_params(axis="both", which="major", pad=1.0, length=2.5)
    for spine in ax.spines.values():
        spine.set_color("#666666")


def last_window(dq: Deque[float] | Deque[int], window_points: int) -> np.ndarray:
    arr = np.asarray(dq, dtype=float)
    if arr.size <= window_points:
        return arr
    return arr[-window_points:]


def set_scroll_xlim(ax, idx_arr: np.ndarray) -> None:
    if idx_arr.size == 0:
        return
    left = idx_arr[0]
    right = idx_arr[-1] if idx_arr.size > 1 else idx_arr[0] + 1
    if right <= left:
        right = left + 1
    ax.set_xlim(left, right)


def autoscale_y(ax, arrays: list[np.ndarray], pad_frac: float = 0.08) -> None:
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


def autoscale_xy_all_points(ax, xs, ys, xds, yds) -> None:
    xs_all = np.asarray(xs, dtype=float)
    ys_all = np.asarray(ys, dtype=float)
    xds_all = np.asarray(xds, dtype=float)
    yds_all = np.asarray(yds, dtype=float)

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
    ax.set_xlim(xmin - pad_x, xmax + pad_x)
    ax.set_ylim(ymin - pad_y, ymax + pad_y)