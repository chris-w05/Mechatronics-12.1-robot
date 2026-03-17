
from __future__ import annotations

import time
from typing import Optional

import serial
from PySide6 import QtCore

from parser import parse_line


class SerialWorker(QtCore.QObject):
    sample_rx = QtCore.Signal(object)
    raw_rx = QtCore.Signal(str)
    status_rx = QtCore.Signal(str)
    stats_rx = QtCore.Signal(int, int)

    def __init__(self):
        super().__init__()
        self._ser: Optional[serial.Serial] = None
        self._timer: Optional[QtCore.QTimer] = None
        self._stats_timer: Optional[QtCore.QTimer] = None
        self._paused = False
        self._track_width_in: Optional[float] = None
        self._prefix = "P"
        self._rx_chunks_accum = 0
        self._parse_err_accum = 0
        self._text_buffer = ""
        self._max_buffer = 16384

    @QtCore.Slot()
    def start(self):
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.poll_serial)
        self._timer.start(5)

        self._stats_timer = QtCore.QTimer(self)
        self._stats_timer.timeout.connect(self.emit_stats)
        self._stats_timer.start(1000)

    @QtCore.Slot(float, str)
    def configure(self, track_width_in: float, prefix: str):
        self._track_width_in = track_width_in if track_width_in > 0 else None
        self._prefix = prefix.strip().upper()

    @QtCore.Slot(str, int)
    def connect_port(self, port: str, baud: int):
        self.disconnect_port()
        try:
            self._ser = serial.Serial(port, baud, timeout=0.0)
            time.sleep(0.2)
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
            self._text_buffer = ""
            self.status_rx.emit(f"Connected: {port} @ {baud}")
        except Exception as e:
            self._ser = None
            self.status_rx.emit(f"Connect failed: {e}")

    @QtCore.Slot()
    def disconnect_port(self):
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._text_buffer = ""
        self.status_rx.emit("Disconnected")

    @QtCore.Slot(bool)
    def set_paused(self, paused: bool):
        self._paused = paused

    @QtCore.Slot(str)
    def send_line(self, text: str):
        if not self._ser:
            self.status_rx.emit("Not connected")
            return
        text = text.rstrip("\r\n") + "\n"
        try:
            self._ser.write(text.encode("utf-8", errors="replace"))
            self.status_rx.emit(f"Sent: {text.rstrip()}")
        except Exception as e:
            self.status_rx.emit(f"Send failed: {e}")

    @QtCore.Slot()
    def poll_serial(self):
        if not self._ser:
            return

        try:
            waiting = self._ser.in_waiting
        except Exception:
            waiting = 0

        if waiting <= 0:
            return

        try:
            raw = self._ser.read(waiting)
        except Exception:
            return

        if not raw:
            return

        self._rx_chunks_accum += 1
        chunk = raw.decode("utf-8", errors="replace")
        self._text_buffer += chunk

        if len(self._text_buffer) > self._max_buffer:
            self._text_buffer = self._text_buffer[-self._max_buffer:]

        self._extract_packets()

    def _extract_packets(self):
        buf = self._text_buffer

        while True:
            start = buf.find("{")
            if start < 0:
                tail = buf.strip()
                if tail:
                    self.raw_rx.emit(tail)
                self._text_buffer = ""
                return

            prefix = buf[:start].strip()
            if prefix:
                self.raw_rx.emit(prefix)

            depth = 0
            in_string = False
            escape = False
            end = None

            for i in range(start, len(buf)):
                c = buf[i]
                if in_string:
                    if escape:
                        escape = False
                    elif c == "\\":
                        escape = True
                    elif c == '"':
                        in_string = False
                    continue

                if c == '"':
                    in_string = True
                elif c == "{":
                    depth += 1
                elif c == "}":
                    depth -= 1
                    if depth == 0:
                        end = i
                        break

            if end is None:
                self._text_buffer = buf[start:]
                return

            packet = buf[start:end + 1]
            self.raw_rx.emit(packet)

            s = parse_line(packet, track_width_in=self._track_width_in)
            if s is None:
                self._parse_err_accum += 1
            elif not self._paused:
                self.sample_rx.emit(s)

            buf = buf[end + 1:]

    @QtCore.Slot()
    def emit_stats(self):
        self.stats_rx.emit(self._rx_chunks_accum, self._parse_err_accum)
        self._rx_chunks_accum = 0
        self._parse_err_accum = 0