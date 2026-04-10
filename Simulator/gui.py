"""
gui.py  –  PySide6 main window for the Robot Simulator

Layout (compact — all panels fit on one screen):
┌──────────────────────────────────────────────────────────────────────┐
│  Toolbar:  [Run] [Pause] [Reset] [Step] [Load Auto]   speed slider │
├────────────────────────────┬─────────────────────────────────────────┤
│                            │  Right tabs: [Monitor] [Controls]      │
│  Field map (matplotlib)    │   Monitor: Status + Sensor panels      │
│  — robot path + pose       │   Controls: Commands + Strategy        │
│  — walls & tape lines      │                                        │
│                            │  (QSplitter — user-resizable)          │
├────────────────────────────┴─────────────────────────────────────────┤
│  Bottom tabs: [Telemetry] [Auto Builder]                            │
│  — Telemetry plots (Velocities, Heading, Miner, Shooter)            │
│  — Autonomous step builder                                          │
└──────────────────────────────────────────────────────────────────────┘
"""

from __future__ import annotations

import math
import sys

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from .robot import Robot, RobotMode
from .environment import FieldEnvironment
from .strategy import Strategy, Block, CycleType, ToolLevel
from . import config as C
from .autonomous import (
    DriveDistanceStep,
    DriveRadiusAngleStep,
    DriveLineUntilWallStep,
    DeployRampStep,
    MineBlockStep,
    FollowLineStepAuto,
    DriveUntilWallStep,
    DelayStepAuto,
)


# ═════════════════════════════════════════════════════════════════════════════
#  Color palette
# ═════════════════════════════════════════════════════════════════════════════
BG = "#0e0e0e"
FG = "#dddddd"
ACCENT = "#3a86ff"
ACCENT2 = "#ff006e"
PANEL_BG = "#161616"
INPUT_BG = "#1a1a1a"

STYLESHEET = f"""
QWidget {{ background-color: {BG}; color: {FG}; font-family: 'Menlo', 'Courier New'; font-size: 12px; }}
QMainWindow {{ background-color: {BG}; }}
QGroupBox {{ border: 1px solid #333; border-radius: 4px; margin-top: 8px; padding-top: 14px; font-weight: bold; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; }}
QPushButton {{ background-color: #1e1e1e; border: 1px solid #444; border-radius: 3px; padding: 5px 14px; }}
QPushButton:hover {{ background-color: #2a2a2a; }}
QPushButton:pressed {{ background-color: {ACCENT}; color: black; }}
QPushButton:checked {{ background-color: {ACCENT}; color: black; }}
QSlider::groove:horizontal {{ height: 6px; background: #333; border-radius: 3px; }}
QSlider::handle:horizontal {{ width: 14px; margin: -4px 0; background: {ACCENT}; border-radius: 7px; }}
QComboBox {{ background-color: {INPUT_BG}; border: 1px solid #444; padding: 3px 8px; border-radius: 3px; }}
QComboBox QAbstractItemView {{ background-color: {INPUT_BG}; color: {FG}; selection-background-color: {ACCENT}; }}
QLabel {{ font-size: 11px; }}
QCheckBox::indicator {{ width: 16px; height: 16px; }}
QCheckBox::indicator:checked {{ background-color: {ACCENT}; border: 1px solid {ACCENT}; border-radius: 3px; }}
QCheckBox::indicator:unchecked {{ background-color: #333; border: 1px solid #555; border-radius: 3px; }}
QTabWidget::pane {{ border: 1px solid #333; }}
QTabBar::tab {{ background: #1a1a1a; border: 1px solid #333; padding: 5px 12px; }}
QTabBar::tab:selected {{ background: {ACCENT}; color: black; }}
QLineEdit {{ background-color: {INPUT_BG}; border: 1px solid #444; border-radius: 3px; padding: 3px 6px; }}
QSpinBox, QDoubleSpinBox {{ background-color: {INPUT_BG}; border: 1px solid #444; border-radius: 3px; padding: 2px 6px; }}
"""


def _dark_figure(width=5, height=4) -> Figure:
    fig = Figure(figsize=(width, height), dpi=100, facecolor=BG)
    return fig


# ═════════════════════════════════════════════════════════════════════════════
#  Field canvas
# ═════════════════════════════════════════════════════════════════════════════

class FieldCanvas(FigureCanvas):
    """Matplotlib canvas showing the field map and robot path."""

    def __init__(self, env: FieldEnvironment, parent=None):
        self.fig = _dark_figure(5, 5)
        super().__init__(self.fig)
        self.env = env
        self.ax = self.fig.add_subplot(111)
        self._setup_axes()
        self.setParent(parent)

    def _setup_axes(self):
        ax = self.ax
        ax.set_facecolor(BG)
        ax.set_xlim(-2, self.env.width + 2)
        ax.set_ylim(-2, self.env.height + 2)
        ax.set_aspect("equal")
        ax.set_xlabel("X (in)", color=FG, fontsize=9)
        ax.set_ylabel("Y (in)", color=FG, fontsize=9)
        ax.tick_params(colors=FG, labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#333")
        ax.grid(True, color="#222", linewidth=0.5)

    def redraw(self, robot: Robot):
        ax = self.ax
        ax.clear()
        self._setup_axes()

        # Draw walls
        for w in self.env.walls:
            ax.plot([w.x1, w.x2], [w.y1, w.y2], color="#888", linewidth=2)

        # Draw tape lines
        for line in self.env.lines:
            if len(line.points) >= 2:
                xs = [p[0] for p in line.points]
                ys = [p[1] for p in line.points]
                ax.plot(xs, ys, color="#ffbe0b", linewidth=1.5, linestyle="--", alpha=0.7)

        # Draw path (desired)
        if len(robot.history_x) > 1:
            ax.plot(robot.history_x, robot.history_y,
                    color=ACCENT, linewidth=1, alpha=0.6, label="Desired")

        # Draw true path
        if len(robot.history_true_x) > 1:
            ax.plot(robot.history_true_x, robot.history_true_y,
                    color=ACCENT2, linewidth=1, alpha=0.6, label="True")

        # Draw robot as triangle
        pose = robot.drive.get_true_pose()
        size = 3.0
        angle = pose.heading
        tx = pose.x + size * math.cos(angle)
        ty = pose.y + size * math.sin(angle)
        lx = pose.x + size * 0.5 * math.cos(angle + 2.4)
        ly = pose.y + size * 0.5 * math.sin(angle + 2.4)
        rx = pose.x + size * 0.5 * math.cos(angle - 2.4)
        ry = pose.y + size * 0.5 * math.sin(angle - 2.4)
        triangle = plt_Polygon([(tx, ty), (lx, ly), (rx, ry)],
                                closed=True, facecolor=ACCENT, edgecolor="white",
                                linewidth=1.5, alpha=0.9, zorder=10)
        ax.add_patch(triangle)

        # Heading line
        hx = pose.x + 5 * math.cos(angle)
        hy = pose.y + 5 * math.sin(angle)
        ax.plot([pose.x, hx], [pose.y, hy], color="white", linewidth=0.5, alpha=0.4)

        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc="upper right", fontsize=7, facecolor=BG, edgecolor="#333",
                      labelcolor=FG)
        self.draw()


# Need Polygon from matplotlib
from matplotlib.patches import Polygon as plt_Polygon


# ═════════════════════════════════════════════════════════════════════════════
#  Telemetry canvas (tabbed plots)
# ═════════════════════════════════════════════════════════════════════════════

class TelemetryCanvas(FigureCanvas):
    """A single matplotlib figure with subplots for telemetry."""

    def __init__(self, parent=None):
        self.fig = _dark_figure(10, 3)
        super().__init__(self.fig)
        self.axes = self.fig.subplots(1, 4)
        self.fig.subplots_adjust(left=0.06, right=0.98, bottom=0.18, top=0.88, wspace=0.35)
        for ax in self.axes:
            ax.set_facecolor(BG)
            ax.tick_params(colors=FG, labelsize=7)
            for spine in ax.spines.values():
                spine.set_color("#333")
            ax.grid(True, color="#222", linewidth=0.4)
        self.axes[0].set_title("Wheel Velocities", color=FG, fontsize=9)
        self.axes[1].set_title("Heading", color=FG, fontsize=9)
        self.axes[2].set_title("Miner Servo", color=FG, fontsize=9)
        self.axes[3].set_title("Shooter Position", color=FG, fontsize=9)
        self.setParent(parent)

    def redraw(self, robot: Robot):
        t = robot.history_t
        for ax in self.axes:
            ax.clear()
            ax.set_facecolor(BG)
            ax.tick_params(colors=FG, labelsize=7)
            for spine in ax.spines.values():
                spine.set_color("#333")
            ax.grid(True, color="#222", linewidth=0.4)

        if not t:
            self.draw()
            return

        self.axes[0].plot(t, robot.history_vel_l, color=ACCENT, linewidth=1, label="Left")
        self.axes[0].plot(t, robot.history_vel_r, color=ACCENT2, linewidth=1, label="Right")
        self.axes[0].set_title("Wheel Velocities (in/s)", color=FG, fontsize=9)
        self.axes[0].legend(fontsize=6, facecolor=BG, edgecolor="#333", labelcolor=FG)
        self.axes[0].set_xlabel("t (s)", color=FG, fontsize=8)

        self.axes[1].plot(t, [h * 180 / math.pi for h in robot.history_heading],
                          color="#8338ec", linewidth=1)
        self.axes[1].set_title("Heading (°)", color=FG, fontsize=9)
        self.axes[1].set_xlabel("t (s)", color=FG, fontsize=8)

        self.axes[2].plot(t, robot.history_miner_angle, color="#fb5607", linewidth=1)
        self.axes[2].set_title("Miner Servo (°)", color=FG, fontsize=9)
        self.axes[2].set_xlabel("t (s)", color=FG, fontsize=8)

        self.axes[3].plot(t, robot.history_shooter_pos, color="#06d6a0", linewidth=1)
        self.axes[3].set_title("Shooter Pos (rot)", color=FG, fontsize=9)
        self.axes[3].set_xlabel("t (s)", color=FG, fontsize=8)

        self.draw()


# ═════════════════════════════════════════════════════════════════════════════
#  Sensor Input Panel
# ═════════════════════════════════════════════════════════════════════════════

class SensorPanel(QtWidgets.QGroupBox):
    """Panel for user-injected sensor values."""

    def __init__(self, parent=None):
        super().__init__("Sensor Inputs", parent)
        layout = QtWidgets.QFormLayout(self)
        layout.setSpacing(8)

        # Line sensor (-5 to 5 cm)
        self.line_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.line_slider.setRange(-500, 500)  # ×100 for precision
        self.line_slider.setValue(0)
        self.line_label = QtWidgets.QLabel("0.00 cm")
        self.line_slider.valueChanged.connect(
            lambda v: self.line_label.setText(f"{v/100:.2f} cm"))
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.line_slider)
        row.addWidget(self.line_label)
        layout.addRow("Line Sensor:", row)

        # Distance sensor (0-30 cm)
        self.dist_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.dist_slider.setRange(0, 3000)  # ×100
        self.dist_slider.setValue(1500)
        self.dist_label = QtWidgets.QLabel("15.00 cm")
        self.dist_slider.valueChanged.connect(
            lambda v: self.dist_label.setText(f"{v/100:.2f} cm"))
        row2 = QtWidgets.QHBoxLayout()
        row2.addWidget(self.dist_slider)
        row2.addWidget(self.dist_label)
        layout.addRow("Distance (IR):", row2)

        # Block switch
        self.block_switch = QtWidgets.QCheckBox("Block present")
        layout.addRow("Block Switch:", self.block_switch)

        # Color sensor (block type dropdown)
        self.color_combo = QtWidgets.QComboBox()
        self.color_combo.addItems(["None", "Wood", "Stone", "Iron", "Diamond"])
        layout.addRow("Color Sensor:", self.color_combo)

        # Auto-environment toggle
        self.auto_env = QtWidgets.QCheckBox("Auto-compute from field")
        self.auto_env.setChecked(True)
        self.auto_env.setToolTip(
            "When checked, line sensor and distance sensor values are calculated\n"
            "from the robot's pose on the field map. Uncheck to use manual inputs."
        )
        layout.addRow("Environment:", self.auto_env)

    @property
    def line_sensor_cm(self) -> float:
        return self.line_slider.value() / 100.0

    @property
    def distance_sensor_cm(self) -> float:
        return self.dist_slider.value() / 100.0

    @property
    def detected_block(self) -> Block:
        idx = self.color_combo.currentIndex() - 1  # -1 = NONE, 0=WOOD...
        return Block(idx)


# ═════════════════════════════════════════════════════════════════════════════
#  Status Panel
# ═════════════════════════════════════════════════════════════════════════════

class StatusPanel(QtWidgets.QGroupBox):
    """Live status readouts."""

    def __init__(self, parent=None):
        super().__init__("Robot Status", parent)
        layout = QtWidgets.QFormLayout(self)
        layout.setSpacing(4)

        self.lbl_time = QtWidgets.QLabel("0.000 s")
        self.lbl_mode = QtWidgets.QLabel("AWAIT")
        self.lbl_pose = QtWidgets.QLabel("(0.0, 0.0)  θ=0.0°")
        self.lbl_velocity = QtWidgets.QLabel("L: 0.0  R: 0.0 in/s")
        self.lbl_step = QtWidgets.QLabel("—")
        self.lbl_step_progress = QtWidgets.QLabel("0 / 0")
        self.lbl_battery = QtWidgets.QLabel(f"{C.BATTERY_VOLTAGE:.1f} V")
        self.lbl_current = QtWidgets.QLabel("0.00 A")
        self.lbl_miner = QtWidgets.QLabel("OFF  hits: 0")
        self.lbl_shooter = QtWidgets.QLabel("OFF  pos: 0.00")
        self.lbl_distance_sensor = QtWidgets.QLabel("—")
        self.lbl_line_sensor = QtWidgets.QLabel("—")

        layout.addRow("Time:", self.lbl_time)
        layout.addRow("Mode:", self.lbl_mode)
        layout.addRow("Pose:", self.lbl_pose)
        layout.addRow("Velocity:", self.lbl_velocity)
        layout.addRow("Auto Step:", self.lbl_step)
        layout.addRow("Progress:", self.lbl_step_progress)
        layout.addRow("Battery:", self.lbl_battery)
        layout.addRow("Current:", self.lbl_current)
        layout.addRow("Miner:", self.lbl_miner)
        layout.addRow("Shooter:", self.lbl_shooter)
        layout.addRow("Dist Sensor:", self.lbl_distance_sensor)
        layout.addRow("Line Sensor:", self.lbl_line_sensor)

    def refresh(self, robot: Robot):
        p = robot.drive.get_true_pose()
        self.lbl_time.setText(f"{robot.sim_time:.3f} s")
        self.lbl_mode.setText(robot.mode.name)
        self.lbl_pose.setText(
            f"({p.x:.2f}, {p.y:.2f})  θ={math.degrees(p.heading):.1f}°"
        )
        lv = robot.drive.left_encoder.get_velocity() * C.DRIVETRAIN_TICKS_TO_IN
        rv = robot.drive.right_encoder.get_velocity() * C.DRIVETRAIN_TICKS_TO_IN
        self.lbl_velocity.setText(f"L: {lv:.1f}  R: {rv:.1f} in/s")
        self.lbl_step.setText(robot.autonomous.current_step_name)
        self.lbl_step_progress.setText(
            f"{robot.autonomous.current_index} / {robot.autonomous.step_count}")
        self.lbl_battery.setText(f"{robot.battery_voltage:.1f} V")
        self.lbl_current.setText(f"{robot.total_current_draw:.2f} A")
        self.lbl_miner.setText(
            f"{robot.miner.mode.name}  hits: {robot.miner.hits_done}")
        self.lbl_shooter.setText(
            f"{robot.shooter.mode.name}  pos: {robot.shooter.position:.3f}")
        self.lbl_distance_sensor.setText(
            f"{robot.drive.distance_sensor_cm:.1f} cm / {robot.drive.distance_sensor_cm/2.54:.2f} in")
        self.lbl_line_sensor.setText(f"{robot.drive.line_sensor_value:.2f} cm")


# ═════════════════════════════════════════════════════════════════════════════
#  Command Panel — for manual serial-like commands
# ═════════════════════════════════════════════════════════════════════════════

class CommandPanel(QtWidgets.QGroupBox):
    """Panel for issuing robot commands (mirrors serial test mode)."""

    command_issued = QtCore.Signal(str, float, float)  # cmd, p1, p2

    def __init__(self, parent=None):
        super().__init__("Commands", parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(4)

        # Quick buttons
        btn_row = QtWidgets.QHBoxLayout()
        for label in ["Stop", "Mine", "Fire", "Prime"]:
            btn = QtWidgets.QPushButton(label)
            btn.clicked.connect(lambda checked, l=label: self.command_issued.emit(l, 0, 0))
            btn_row.addWidget(btn)
        layout.addLayout(btn_row)

        # Drive speed
        drive_row = QtWidgets.QHBoxLayout()
        drive_row.addWidget(QtWidgets.QLabel("Drive (in/s):"))
        self.drive_spin = QtWidgets.QDoubleSpinBox()
        self.drive_spin.setRange(-27, 27)
        self.drive_spin.setValue(10)
        self.drive_spin.setSingleStep(1)
        drive_row.addWidget(self.drive_spin)
        drive_btn = QtWidgets.QPushButton("Go")
        drive_btn.clicked.connect(
            lambda: self.command_issued.emit("Drive", self.drive_spin.value(), 0))
        drive_row.addWidget(drive_btn)
        layout.addLayout(drive_row)

        # Arc
        arc_row = QtWidgets.QHBoxLayout()
        arc_row.addWidget(QtWidgets.QLabel("Arc r(in):"))
        self.arc_r = QtWidgets.QDoubleSpinBox()
        self.arc_r.setRange(-50, 50)
        self.arc_r.setValue(15)
        arc_row.addWidget(self.arc_r)
        arc_row.addWidget(QtWidgets.QLabel("°:"))
        self.arc_deg = QtWidgets.QDoubleSpinBox()
        self.arc_deg.setRange(-360, 360)
        self.arc_deg.setValue(90)
        arc_row.addWidget(self.arc_deg)
        arc_btn = QtWidgets.QPushButton("Arc")
        arc_btn.clicked.connect(
            lambda: self.command_issued.emit("Arc", self.arc_r.value(), self.arc_deg.value()))
        arc_row.addWidget(arc_btn)
        layout.addLayout(arc_row)

        # Line follow
        lf_row = QtWidgets.QHBoxLayout()
        lf_row.addWidget(QtWidgets.QLabel("Linefollow PWM:"))
        self.lf_spin = QtWidgets.QSpinBox()
        self.lf_spin.setRange(0, 400)
        self.lf_spin.setValue(200)
        lf_row.addWidget(self.lf_spin)
        lf_btn = QtWidgets.QPushButton("Follow")
        lf_btn.clicked.connect(
            lambda: self.command_issued.emit("Linefollow", self.lf_spin.value(), 0))
        lf_row.addWidget(lf_btn)
        layout.addLayout(lf_row)


# ═════════════════════════════════════════════════════════════════════════════
#  Autonomous Builder Panel
# ═════════════════════════════════════════════════════════════════════════════

class AutoBuilderPanel(QtWidgets.QGroupBox):
    """Panel for building custom autonomous step sequences."""

    routine_changed = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__("Autonomous Builder", parent)
        layout = QtWidgets.QVBoxLayout(self)

        # Step type selector
        type_row = QtWidgets.QHBoxLayout()
        type_row.addWidget(QtWidgets.QLabel("Step:"))
        self.step_combo = QtWidgets.QComboBox()
        self.step_combo.addItems([
            "DriveDistance", "DriveRadiusAngle", "DriveUntilWall",
            "DriveLineUntilWall", "FollowLine", "MineBlock",
            "DeployRamp", "Delay",
        ])
        type_row.addWidget(self.step_combo)
        layout.addLayout(type_row)

        # Parameters
        param_row = QtWidgets.QHBoxLayout()
        param_row.addWidget(QtWidgets.QLabel("P1:"))
        self.p1 = QtWidgets.QDoubleSpinBox()
        self.p1.setRange(-1000, 1000)
        self.p1.setValue(18)
        param_row.addWidget(self.p1)
        param_row.addWidget(QtWidgets.QLabel("P2:"))
        self.p2 = QtWidgets.QDoubleSpinBox()
        self.p2.setRange(-1000, 1000)
        self.p2.setValue(15)
        param_row.addWidget(self.p2)
        param_row.addWidget(QtWidgets.QLabel("P3:"))
        self.p3 = QtWidgets.QDoubleSpinBox()
        self.p3.setRange(-1000, 1000)
        self.p3.setValue(45)
        param_row.addWidget(self.p3)
        layout.addLayout(param_row)

        # Add/Remove/Default buttons
        btn_row = QtWidgets.QHBoxLayout()
        add_btn = QtWidgets.QPushButton("Add Step")
        add_btn.clicked.connect(self._add_step)
        btn_row.addWidget(add_btn)
        remove_btn = QtWidgets.QPushButton("Remove Selected")
        remove_btn.clicked.connect(self._remove_step)
        btn_row.addWidget(remove_btn)
        default_btn = QtWidgets.QPushButton("Load Default")
        default_btn.clicked.connect(self._load_default)
        btn_row.addWidget(default_btn)
        clear_btn = QtWidgets.QPushButton("Clear")
        clear_btn.clicked.connect(self._clear)
        btn_row.addWidget(clear_btn)
        layout.addLayout(btn_row)

        # Step list
        self.step_list = QtWidgets.QListWidget()
        self.step_list.setMaximumHeight(140)
        layout.addWidget(self.step_list)

        # Internal step descriptors
        self._step_descriptors: list[tuple[str, float, float, float]] = []

    def _add_step(self):
        step_type = self.step_combo.currentText()
        p1, p2, p3 = self.p1.value(), self.p2.value(), self.p3.value()
        self._step_descriptors.append((step_type, p1, p2, p3))
        self._refresh_list()
        self.routine_changed.emit()

    def _remove_step(self):
        row = self.step_list.currentRow()
        if 0 <= row < len(self._step_descriptors):
            self._step_descriptors.pop(row)
            self._refresh_list()
            self.routine_changed.emit()

    def _load_default(self):
        speed = 15.0
        self._step_descriptors = [
            ("DriveDistance", 18, speed, 0),
            ("DriveRadiusAngle", speed, -20, 45),
            ("DriveRadiusAngle", speed, 20, -45),
            ("DriveDistance", 23, speed, 0),
            ("DriveRadiusAngle", speed * 0.5, -15, 90),
            ("DriveDistance", 2, speed * 0.25, 0),
            ("DriveLineUntilWall", 0, 0, 0),
            ("DeployRamp", 0, 0, 0),
            ("MineBlock", 1000, 0, 0),
        ]
        self._refresh_list()
        self.routine_changed.emit()

    def _clear(self):
        self._step_descriptors.clear()
        self._refresh_list()
        self.routine_changed.emit()

    def _refresh_list(self):
        self.step_list.clear()
        for i, (st, p1, p2, p3) in enumerate(self._step_descriptors):
            self.step_list.addItem(f"{i+1}. {st}({p1:.1f}, {p2:.1f}, {p3:.1f})")

    def build_routine(self, drive, miner, shooter) -> list:
        """Create AutoStep objects from the descriptor list."""
        steps = []
        for (st, p1, p2, p3) in self._step_descriptors:
            if st == "DriveDistance":
                steps.append(DriveDistanceStep(drive, p1, p2))
            elif st == "DriveRadiusAngle":
                steps.append(DriveRadiusAngleStep(drive, p1, p2, p3))
            elif st == "DriveUntilWall":
                steps.append(DriveUntilWallStep(drive, p1))
            elif st == "DriveLineUntilWall":
                steps.append(DriveLineUntilWallStep(drive, p1))
            elif st == "FollowLine":
                steps.append(FollowLineStepAuto(drive, p1, int(p2)))
            elif st == "MineBlock":
                steps.append(MineBlockStep(miner, int(p1)))
            elif st == "DeployRamp":
                steps.append(DeployRampStep(miner))
            elif st == "Delay":
                steps.append(DelayStepAuto(int(p1)))
        return steps


# ═════════════════════════════════════════════════════════════════════════════
#  Strategy Panel
# ═════════════════════════════════════════════════════════════════════════════

class StrategyPanel(QtWidgets.QGroupBox):
    """Shows current game strategy state."""

    def __init__(self, strategy: Strategy, parent=None):
        super().__init__("Strategy", parent)
        self.strategy = strategy
        layout = QtWidgets.QFormLayout(self)

        self.lbl_pickaxe = QtWidgets.QLabel("WOOD")
        self.lbl_sword = QtWidgets.QLabel("WOOD")
        self.lbl_cycle = QtWidgets.QLabel("PICKAXE_HANDLE")
        self.lbl_possession = QtWidgets.QLabel("[—, —, —]")
        self.lbl_max_block = QtWidgets.QLabel("STONE")

        layout.addRow("Pickaxe:", self.lbl_pickaxe)
        layout.addRow("Sword:", self.lbl_sword)
        layout.addRow("Next Item:", self.lbl_cycle)
        layout.addRow("Possession:", self.lbl_possession)
        layout.addRow("Max Block:", self.lbl_max_block)

    def refresh(self):
        s = self.strategy
        self.lbl_pickaxe.setText(ToolLevel(s.pickaxe_level).name)
        self.lbl_sword.setText(ToolLevel(s.sword_level).name)
        self.lbl_cycle.setText(CycleType(s.next_item).name)
        names = [Block(b).name for b in s.possession]
        self.lbl_possession.setText(f"[{', '.join(names)}]")
        self.lbl_max_block.setText(s.get_max_mineable_block().name)


# ═════════════════════════════════════════════════════════════════════════════
#  Main Window
# ═════════════════════════════════════════════════════════════════════════════

class SimulatorWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Simulator — ME EN 3230")
        self.resize(1400, 900)

        # Core objects
        self.robot = Robot()
        self.robot.init()
        self.env = FieldEnvironment()
        self.strategy = Strategy()

        # Sim control
        self._running = False
        self._sim_speed = 1.0     # multiplier
        self._steps_per_tick = 10  # sim steps per timer tick
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.setInterval(16)  # ~60 fps

        self._build_ui()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setSpacing(4)

        # ── Toolbar ──────────────────────────────────────────────────────────
        toolbar = QtWidgets.QHBoxLayout()

        self.btn_run = QtWidgets.QPushButton("▶ Run")
        self.btn_run.setCheckable(True)
        self.btn_run.clicked.connect(self._toggle_run)
        toolbar.addWidget(self.btn_run)

        btn_step = QtWidgets.QPushButton("⏭ Step")
        btn_step.clicked.connect(self._step_once)
        toolbar.addWidget(btn_step)

        btn_step10 = QtWidgets.QPushButton("⏩ ×100")
        btn_step10.clicked.connect(lambda: self._step_n(100))
        toolbar.addWidget(btn_step10)

        btn_reset = QtWidgets.QPushButton("⟲ Reset")
        btn_reset.clicked.connect(self._reset)
        toolbar.addWidget(btn_reset)

        btn_auto = QtWidgets.QPushButton("▶ Start Auto")
        btn_auto.clicked.connect(self._start_auto)
        toolbar.addWidget(btn_auto)

        btn_stop = QtWidgets.QPushButton("⏹ Stop All")
        btn_stop.clicked.connect(self._stop_all)
        toolbar.addWidget(btn_stop)

        toolbar.addStretch()

        toolbar.addWidget(QtWidgets.QLabel("Speed:"))
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(10)
        self.speed_slider.setFixedWidth(150)
        self.speed_label = QtWidgets.QLabel("×1.0")
        self.speed_slider.valueChanged.connect(self._update_speed)
        toolbar.addWidget(self.speed_slider)
        toolbar.addWidget(self.speed_label)

        root.addLayout(toolbar)

        # ── Main area: field + side panels ───────────────────────────────────
        main_split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        # Field map
        self.field_canvas = FieldCanvas(self.env)
        self.field_canvas.setMinimumSize(300, 300)
        main_split.addWidget(self.field_canvas)

        # Right side: tabbed panels
        right_tabs = QtWidgets.QTabWidget()

        # "Monitor" tab — Status + Sensors
        monitor_widget = QtWidgets.QWidget()
        monitor_layout = QtWidgets.QVBoxLayout(monitor_widget)
        monitor_layout.setContentsMargins(0, 0, 0, 0)
        monitor_layout.setSpacing(2)
        self.status_panel = StatusPanel()
        monitor_layout.addWidget(self.status_panel)
        self.sensor_panel = SensorPanel()
        monitor_layout.addWidget(self.sensor_panel)
        monitor_layout.addStretch()
        right_tabs.addTab(monitor_widget, "Monitor")

        # "Controls" tab — Commands + Strategy
        controls_widget = QtWidgets.QWidget()
        controls_layout = QtWidgets.QVBoxLayout(controls_widget)
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.setSpacing(2)
        self.command_panel = CommandPanel()
        self.command_panel.command_issued.connect(self._handle_command)
        controls_layout.addWidget(self.command_panel)
        self.strategy_panel = StrategyPanel(self.strategy)
        controls_layout.addWidget(self.strategy_panel)
        controls_layout.addStretch()
        right_tabs.addTab(controls_widget, "Controls")

        main_split.addWidget(right_tabs)
        main_split.setStretchFactor(0, 3)
        main_split.setStretchFactor(1, 2)

        # ── Bottom section: Auto Builder + Telemetry in tabs ─────────────────
        bottom_tabs = QtWidgets.QTabWidget()

        self.telemetry = TelemetryCanvas()
        bottom_tabs.addTab(self.telemetry, "Telemetry")

        self.auto_builder = AutoBuilderPanel()
        bottom_tabs.addTab(self.auto_builder, "Auto Builder")

        bottom_tabs.setMinimumHeight(180)

        # Vertical splitter: main area on top, bottom tabs below
        vsplit = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        vsplit.addWidget(main_split)
        vsplit.addWidget(bottom_tabs)
        vsplit.setStretchFactor(0, 3)
        vsplit.setStretchFactor(1, 1)

        root.addWidget(vsplit, stretch=1)

    # ── Timer / simulation loop ──────────────────────────────────────────────

    def _toggle_run(self):
        self._running = self.btn_run.isChecked()
        if self._running:
            self.btn_run.setText("⏸ Pause")
            self._timer.start()
        else:
            self.btn_run.setText("▶ Run")
            self._timer.stop()
            self._redraw()

    def _tick(self):
        if not self._running:
            return
        for _ in range(self._steps_per_tick):
            self._apply_sensors()
            self.robot.step()
        self._redraw()

    def _step_once(self):
        self._apply_sensors()
        self.robot.step()
        self._redraw()

    def _step_n(self, n: int):
        for _ in range(n):
            self._apply_sensors()
            self.robot.step()
        self._redraw()

    def _reset(self):
        self._running = False
        self.btn_run.setChecked(False)
        self.btn_run.setText("▶ Run")
        self._timer.stop()
        self.robot = Robot()
        self.robot.init()
        self._redraw()

    def _start_auto(self):
        # Build from auto builder panel
        steps = self.auto_builder.build_routine(
            self.robot.drive, self.robot.miner, self.robot.shooter
        )
        self.robot.autonomous.clear()
        if steps:
            for s in steps:
                self.robot.autonomous.add(s)
        else:
            self.robot.load_default_autonomous()
        self.robot.start_autonomous()

    def _stop_all(self):
        self.robot.stop_all()
        self._running = False
        self.btn_run.setChecked(False)
        self.btn_run.setText("▶ Run")
        self._timer.stop()
        self._redraw()

    def _update_speed(self, val):
        self._sim_speed = val / 10.0
        self._steps_per_tick = max(1, int(self._sim_speed * 10))
        self.speed_label.setText(f"×{self._sim_speed:.1f}")

    def _apply_sensors(self):
        """Feed sensor values into the robot from user inputs or environment."""
        if self.sensor_panel.auto_env.isChecked():
            pose = self.robot.drive.get_true_pose()
            dist = self.env.distance_to_wall_ahead(pose)
            line = self.env.line_sensor_offset(pose)
            self.robot.drive.distance_sensor_cm = dist * 2.54  # convert in→cm
            self.robot.drive.line_sensor_value = line
            # Update slider displays to show computed values
            self.sensor_panel.dist_slider.blockSignals(True)
            self.sensor_panel.dist_slider.setValue(int(dist * 2.54 * 100))
            self.sensor_panel.dist_label.setText(f"{dist*2.54:.2f} cm")
            self.sensor_panel.dist_slider.blockSignals(False)
            self.sensor_panel.line_slider.blockSignals(True)
            self.sensor_panel.line_slider.setValue(int(line * 100))
            self.sensor_panel.line_label.setText(f"{line:.2f} cm")
            self.sensor_panel.line_slider.blockSignals(False)
        else:
            self.robot.drive.line_sensor_value = self.sensor_panel.line_sensor_cm
            self.robot.drive.distance_sensor_cm = self.sensor_panel.distance_sensor_cm

        self.robot.shooter.block_switch = self.sensor_panel.block_switch.isChecked()

    def _redraw(self):
        self.field_canvas.redraw(self.robot)
        self.status_panel.refresh(self.robot)
        self.strategy_panel.refresh()
        # Throttle telemetry redraws (expensive)
        if len(self.robot.history_t) % 50 == 0 or not self._running:
            self.telemetry.redraw(self.robot)

    # ── Command handling ─────────────────────────────────────────────────────

    def _handle_command(self, cmd: str, p1: float, p2: float):
        r = self.robot
        r.mode = RobotMode.SERIAL_TEST
        if cmd == "Stop":
            r.stop_all()
        elif cmd == "Mine":
            r.miner.start_mining(-1)
        elif cmd == "Fire":
            r.shooter.auto_fire()
        elif cmd == "Prime":
            r.shooter.prime()
        elif cmd == "Drive":
            r.drive.set_speed(p1)
        elif cmd == "Linefollow":
            r.drive.follow_line_hardset(int(p1))
        elif cmd == "Arc":
            # Build one-step arc routine
            r.autonomous.clear()
            from .autonomous import DriveRadiusAngleStep
            r.autonomous.add(DriveRadiusAngleStep(r.drive, 10, p1, p2))
            r.autonomous.start()
