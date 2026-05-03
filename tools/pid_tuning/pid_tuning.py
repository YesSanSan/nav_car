import collections
import json
import queue
import struct
import sys
import threading
import time
from pathlib import Path

import bisect
import math
import serial
from serial.tools import list_ports

try:
    from PySide6 import QtCore, QtGui, QtWidgets
    import pyqtgraph as pg
except Exception as exc:
    print("缺少依赖，请先运行: pip install -r requirements.txt", file=sys.stderr)
    print(f"导入错误: {exc}", file=sys.stderr)
    raise


SERIAL_MEMORY_FILE = Path(__file__).with_name(".pid_tuning_port.json")
BAUD_RATE = 921600
MAX_MOTORS = 4
MAX_TELEMETRY_SAMPLES_PER_MOTOR = 10000
DEFAULT_X_POINTS = 1000
SAVE_SET_PID_ACK_TIMEOUT = 1.0
SAVE_FLASH_ACK_TIMEOUT = 3.0
SAVE_MAX_RETRIES = 2

FRAME_HEADER = 0xA5

TYPE_SET_PID = 0x01
TYPE_SAVE_PID = 0x02
TYPE_LOAD_PID = 0x03
TYPE_READ_PID = 0x04
TYPE_DEFAULT_PID = 0x05
TYPE_SET_SPEED = 0x06
TYPE_PAUSE = 0x07
TYPE_ENTER_BOOTLOADER = 0x08
TYPE_TELEMETRY_KEEPALIVE = 0x09
TYPE_SET_TELEMETRY_CONFIG = 0x0A

TYPE_ACK = 0x80
TYPE_PID_PARAMS = 0x81
TYPE_MOTOR_TELEMETRY = 0x82

CRC16_INIT = 0xFFFF

SIGNALS = [
    ("target_rpm", "Target RPM"),
    ("real_target_rpm", "Ramp Target"),
    ("speed_rpm", "Speed RPM"),
    ("pid_integral", "Integral"),
    ("pwm_output", "PWM"),
    ("compensated_pwm_output", "PWM Comp"),
    ("battery_voltage", "Voltage"),
]
SIGNAL_BITS = {name: 1 << index for index, (name, _label) in enumerate(SIGNALS)}

CHART_PRESETS = [
    ("速度", ("target_rpm", "real_target_rpm", "speed_rpm")),
    ("PWM", ("pwm_output", "compensated_pwm_output")),
    ("积分", ("pid_integral",)),
    ("电压", ("battery_voltage",)),
]


def load_last_serial_port():
    try:
        with SERIAL_MEMORY_FILE.open("r", encoding="utf-8") as file:
            data = json.load(file)
    except (OSError, json.JSONDecodeError):
        return None
    port = data.get("port")
    return port if isinstance(port, str) and port else None


def save_last_serial_port(port):
    try:
        with SERIAL_MEMORY_FILE.open("w", encoding="utf-8") as file:
            json.dump({"port": port}, file, ensure_ascii=False, indent=2)
    except OSError:
        pass


def get_serial_ports():
    return sorted(list_ports.comports(), key=lambda port: port.device.casefold())


def serial_port_label(port_info, last_port):
    parts = [port_info.device]
    if port_info.description and port_info.description != "n/a":
        parts.append(port_info.description)
    if port_info.device == last_port:
        parts.append("上次使用")
    return " - ".join(parts)


def crc16(data: bytes) -> int:
    crc = CRC16_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def pack_frame(frame_type: int, payload: bytes = b"") -> bytes:
    length = 1 + 1 + 1 + len(payload) + 2
    frame = bytearray([FRAME_HEADER, length, frame_type])
    frame.extend(payload)
    frame.extend(struct.pack("<H", crc16(frame)))
    return bytes(frame)


def safe_float(widget, default=0.0):
    try:
        text = widget.text().strip()
        return float(text) if text else default
    except ValueError:
        return default


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("四轮PID调参工具")
        self.resize(1280, 800)

        self.ser = None
        self.running = False
        self.reader_running = False
        self.active_motor_count = MAX_MOTORS
        self.rx_buffer = bytearray()
        self.telemetry_lock = threading.Lock()
        self.telemetry_times = {
            name: [
                collections.deque(maxlen=MAX_TELEMETRY_SAMPLES_PER_MOTOR)
                for _ in range(MAX_MOTORS)
            ]
            for name, _label in SIGNALS
        }
        self.telemetry_values = {
            name: [
                collections.deque(maxlen=MAX_TELEMETRY_SAMPLES_PER_MOTOR)
                for _ in range(MAX_MOTORS)
            ]
            for name, _label in SIGNALS
        }
        self.latest_telemetry_sequence = -1
        self.latest_telemetry_time_us = 0
        self.valid_frame_times = collections.deque(maxlen=5000)
        self.event_queue = queue.Queue()
        self.telemetry_queue = queue.Queue()
        self.plot_frame_times = collections.deque(maxlen=120)
        self.plots = []
        self.curves = {}
        self.curve_visible = {}
        self.telemetry_version = 0
        self.last_plotted_version = 0
        self.last_plot_version_delta = 0
        self.last_status_update = 0.0
        self.last_y_range = None
        self.last_plot_profile = ""
        self.last_plot_tick = None
        self.max_plot_tick_gap_ms = 0.0
        self.rx_bytes = 0
        self.rx_frames = 0
        self.rx_crc_errors = 0
        self.rx_short_yields = 0
        self.invalid_motor_frames = 0
        self.telemetry_queue_max_depth = 0
        self.last_sent_motor_mask = None
        self.last_sent_signal_mask = None
        self.save_transaction = None
        self.sample_index = 0

        self._build_ui()
        self._build_timers()
        self.refresh_serial_ports()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        controls = QtWidgets.QWidget()
        controls.setMinimumWidth(420)
        controls.setMaximumWidth(520)
        controls_layout = QtWidgets.QVBoxLayout(controls)
        controls_layout.setContentsMargins(0, 0, 0, 0)
        root.addWidget(controls)

        serial_group = QtWidgets.QGroupBox("串口")
        serial_layout = QtWidgets.QGridLayout(serial_group)
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports_button = QtWidgets.QPushButton("刷新")
        self.connect_button = QtWidgets.QPushButton("连接")
        self.disconnect_button = QtWidgets.QPushButton("断开")
        self.uart_hint = QtWidgets.QLabel("固件调参 UART 由 PID_TUNING_USE_UART1/UART3 编译开关决定")
        self.uart_hint.setWordWrap(True)
        self.disconnect_button.setEnabled(False)
        serial_layout.addWidget(self.port_combo, 0, 0, 1, 3)
        serial_layout.addWidget(self.refresh_ports_button, 1, 0)
        serial_layout.addWidget(self.connect_button, 1, 1)
        serial_layout.addWidget(self.disconnect_button, 1, 2)
        serial_layout.addWidget(self.uart_hint, 2, 0, 1, 3)
        self.refresh_ports_button.clicked.connect(self.refresh_serial_ports)
        self.connect_button.clicked.connect(self.connect_selected_serial)
        self.disconnect_button.clicked.connect(self.disconnect_serial)
        controls_layout.addWidget(serial_group)

        pid_group = QtWidgets.QGroupBox("PID")
        pid_grid = QtWidgets.QGridLayout(pid_group)
        self.entry_kp = []
        self.entry_ki = []
        self.entry_kd = []
        for i in range(MAX_MOTORS):
            pid_grid.addWidget(QtWidgets.QLabel(f"轮{i} Kp"), i, 0)
            kp = QtWidgets.QLineEdit("0.0")
            ki = QtWidgets.QLineEdit("0.0")
            kd = QtWidgets.QLineEdit("0.0")
            kp.setMinimumWidth(78)
            ki.setMinimumWidth(78)
            kd.setMinimumWidth(78)
            pid_grid.addWidget(kp, i, 1)
            pid_grid.addWidget(QtWidgets.QLabel("Ki"), i, 2)
            pid_grid.addWidget(ki, i, 3)
            pid_grid.addWidget(QtWidgets.QLabel("Kd"), i, 4)
            pid_grid.addWidget(kd, i, 5)
            self.entry_kp.append(kp)
            self.entry_ki.append(ki)
            self.entry_kd.append(kd)
        controls_layout.addWidget(pid_group)

        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(QtWidgets.QLabel("速度期望值 (m/s)"))
        self.entry_speed = QtWidgets.QLineEdit("0.0")
        self.entry_speed.setMaximumWidth(80)
        speed_row.addWidget(self.entry_speed)
        controls_layout.addLayout(speed_row)

        button_grid = QtWidgets.QGridLayout()
        buttons = [
            ("启动", self.start),
            ("停止", self.stop),
            ("保存到Flash", self.save_pid_to_flash),
            ("从Flash读取", self.load_pid_from_flash),
            ("读取当前", self.read_pid_from_controller),
            ("恢复默认", self.load_default_pid),
        ]
        for i, (text, slot) in enumerate(buttons):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(slot)
            button_grid.addWidget(button, i // 2, i % 2)
        controls_layout.addLayout(button_grid)

        plot_group = QtWidgets.QGroupBox("曲线")
        plot_layout = QtWidgets.QVBoxLayout(plot_group)
        range_grid = QtWidgets.QGridLayout()
        self.x_points = QtWidgets.QSpinBox()
        self.x_points.setRange(50, MAX_TELEMETRY_SAMPLES_PER_MOTOR)
        self.x_points.setValue(DEFAULT_X_POINTS)
        self.x_points.setSingleStep(50)
        self.x_points.setSuffix(" 点")
        self.y_auto = QtWidgets.QCheckBox("Y自适应")
        self.y_auto.setChecked(True)
        self.y_min = QtWidgets.QDoubleSpinBox()
        self.y_min.setRange(-1000000.0, 1000000.0)
        self.y_min.setValue(-1000.0)
        self.y_max = QtWidgets.QDoubleSpinBox()
        self.y_max.setRange(-1000000.0, 1000000.0)
        self.y_max.setValue(1000.0)
        range_grid.addWidget(QtWidgets.QLabel("X点数"), 0, 0)
        range_grid.addWidget(self.x_points, 0, 1)
        range_grid.addWidget(self.y_auto, 0, 2)
        range_grid.addWidget(QtWidgets.QLabel("Y最小"), 1, 0)
        range_grid.addWidget(self.y_min, 1, 1)
        range_grid.addWidget(QtWidgets.QLabel("Y最大"), 1, 2)
        range_grid.addWidget(self.y_max, 1, 3)
        plot_layout.addLayout(range_grid)

        self.signal_checks = {}
        signal_grid = QtWidgets.QGridLayout()
        for i, (name, label) in enumerate(SIGNALS):
            check = QtWidgets.QCheckBox(label)
            check.setChecked(name in {"target_rpm", "speed_rpm", "pwm_output"})
            check.toggled.connect(self.telemetry_selection_changed)
            self.signal_checks[name] = check
            signal_grid.addWidget(check, i // 2, i % 2)
        plot_layout.addLayout(signal_grid)

        self.motor_checks = []
        motor_row = QtWidgets.QHBoxLayout()
        for i in range(MAX_MOTORS):
            check = QtWidgets.QCheckBox(f"M{i}")
            check.setChecked(True)
            check.toggled.connect(self.telemetry_selection_changed)
            self.motor_checks.append(check)
            motor_row.addWidget(check)
        plot_layout.addLayout(motor_row)
        controls_layout.addWidget(plot_group)

        controls_layout.addStretch(1)
        self.status_label = QtWidgets.QLabel("未连接")
        self.status_label.setWordWrap(True)
        controls_layout.addWidget(self.status_label)

        self.plot_area = QtWidgets.QWidget()
        self.plot_layout = QtWidgets.QVBoxLayout(self.plot_area)
        self.plot_layout.setContentsMargins(0, 0, 0, 0)
        self.plot_layout.setSpacing(6)
        root.addWidget(self.plot_area, 1)

        palette = [
            "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
            "#9467bd", "#8c564b", "#e377c2", "#17becf",
        ]
        for motor_index in range(MAX_MOTORS):
            for signal_index, (name, label) in enumerate(SIGNALS):
                color = palette[(motor_index + signal_index) % len(palette)]
                style = QtCore.Qt.PenStyle.SolidLine if signal_index < 3 else QtCore.Qt.PenStyle.DashLine
                pen = pg.mkPen(color=color, width=1.4, style=style)
                self.curves[(motor_index, name)] = []
                self.curve_visible[(motor_index, name)] = []
        for title, signal_names in CHART_PRESETS:
            self.add_plot(title, signal_names)
        self.update_plot_visibility()

    def add_plot(self, title, signal_names):
        plot = pg.PlotWidget()
        plot.setBackground("w")
        plot.showGrid(x=True, y=True, alpha=0.28)
        plot.setLabel("bottom", "Sample")
        plot.setTitle(title)
        plot.addLegend(offset=(8, 8))
        plot.enableAutoRange(axis=pg.ViewBox.XAxis, enable=False)
        plot.setClipToView(True)
        plot.setDownsampling(auto=True, mode="peak")
        plot.setMinimumHeight(180)
        self.plot_layout.addWidget(plot, 1)
        plot_info = {
            "plot": plot,
            "signals": set(signal_names),
            "last_y_range": None,
        }
        self.plots.append(plot_info)

        palette = [
            "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
            "#9467bd", "#8c564b", "#e377c2", "#17becf",
        ]
        for motor_index in range(MAX_MOTORS):
            for signal_index, (name, label) in enumerate(SIGNALS):
                if name not in plot_info["signals"]:
                    continue
                color = palette[(motor_index + signal_index) % len(palette)]
                style = QtCore.Qt.PenStyle.SolidLine if signal_index < 3 else QtCore.Qt.PenStyle.DashLine
                pen = pg.mkPen(color=color, width=1.4, style=style)
                curve = plot.plot([], [], pen=pen, name=f"M{motor_index} {label}", skipFiniteCheck=True)
                curve.setClipToView(True)
                curve.setDownsampling(auto=True, method="peak")
                self.curves[(motor_index, name)].append(curve)
                self.curve_visible[(motor_index, name)].append(True)

    def _build_timers(self):
        self.event_timer = QtCore.QTimer(self)
        self.event_timer.timeout.connect(self.process_events)
        self.event_timer.start(10)

        self.keepalive_timer = QtCore.QTimer(self)
        self.keepalive_timer.timeout.connect(lambda: self.send_frame(TYPE_TELEMETRY_KEEPALIVE))
        self.keepalive_timer.start(500)

        self.pid_speed_timer = QtCore.QTimer(self)
        self.pid_speed_timer.timeout.connect(self.send_pid_speed)
        self.pid_speed_timer.start(200)

        self.telemetry_config_timer = QtCore.QTimer(self)
        self.telemetry_config_timer.timeout.connect(self.send_telemetry_config_if_needed)
        self.telemetry_config_timer.start(200)

        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(16)

    def refresh_serial_ports(self):
        last_port = load_last_serial_port()
        current = self.port_combo.currentData() if hasattr(self, "port_combo") else None
        ports = get_serial_ports()
        self.port_combo.clear()
        for port_info in ports:
            self.port_combo.addItem(serial_port_label(port_info, last_port), port_info.device)
        preferred = current or last_port
        if preferred:
            for index in range(self.port_combo.count()):
                if self.port_combo.itemData(index) == preferred:
                    self.port_combo.setCurrentIndex(index)
                    break
        self.connect_button.setEnabled(self.port_combo.count() > 0 and not self.reader_running)

    def connect_selected_serial(self):
        port = self.port_combo.currentData()
        if not port:
            return
        self.init_serial(port)

    def init_serial(self, port):
        if self.reader_running:
            return
        try:
            serial_kwargs = {"port": port, "baudrate": BAUD_RATE, "timeout": 0.001}
            if sys.platform.startswith("linux"):
                serial_kwargs["exclusive"] = True
            self.ser = serial.Serial(**serial_kwargs)
            save_last_serial_port(port)
            self.reader_running = True
            threading.Thread(target=self.serial_reader, daemon=True).start()
            self.set_status(f"串口已打开: {port}")
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
            self.send_frame(TYPE_READ_PID)
            self.send_telemetry_config_if_needed(force=True)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "串口错误", str(exc))
            self.disconnect_serial()

    def disconnect_serial(self):
        self.running = False
        self.reader_running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.connect_button.setEnabled(self.port_combo.count() > 0)
        self.disconnect_button.setEnabled(False)
        self.set_status("串口已断开")

    def send_frame(self, frame_type: int, payload: bytes = b""):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(pack_frame(frame_type, payload))
        except Exception as exc:
            self.event_queue.put(("status", f"串口写入错误: {exc}"))

    def send_set_pid(self, index: int, kp: float, ki: float, kd: float):
        self.send_frame(TYPE_SET_PID, struct.pack("<B3xfff", index, kp, ki, kd))

    def current_motor_mask(self):
        mask = 0
        for index, check in enumerate(self.motor_checks):
            if check.isChecked():
                mask |= 1 << index
        return mask

    def current_signal_mask(self):
        mask = 0
        for name, check in self.signal_checks.items():
            if check.isChecked():
                mask |= SIGNAL_BITS[name]
        return mask

    def send_telemetry_config_if_needed(self, force=False):
        motor_mask = self.current_motor_mask()
        signal_mask = self.current_signal_mask()
        if not force and motor_mask == self.last_sent_motor_mask and signal_mask == self.last_sent_signal_mask:
            return
        self.last_sent_motor_mask = motor_mask
        self.last_sent_signal_mask = signal_mask
        self.send_frame(TYPE_SET_TELEMETRY_CONFIG, struct.pack("<BBH", motor_mask, signal_mask, 0))

    def telemetry_selection_changed(self):
        self.clear_telemetry_history()
        self.update_plot_visibility()
        self.send_telemetry_config_if_needed(force=True)

    def clear_telemetry_history(self):
        for name, _label in SIGNALS:
            for motor_index in range(MAX_MOTORS):
                self.telemetry_times[name][motor_index].clear()
                self.telemetry_values[name][motor_index].clear()
                for curve in self.curves[(motor_index, name)]:
                    curve.setData([], [])
        self.latest_telemetry_time_us = 0
        self.sample_index = 0
        self.telemetry_version += 1
        self.last_plotted_version = -1

    def update_plot_visibility(self):
        for plot_info in self.plots:
            has_selected_signal = any(self.signal_checks[name].isChecked() for name in plot_info["signals"])
            has_selected_motor = any(check.isChecked() for check in self.motor_checks)
            has_data = any(
                self.telemetry_values[name][motor_index]
                for name in plot_info["signals"]
                for motor_index, check in enumerate(self.motor_checks)
                if check.isChecked()
            )
            plot_info["plot"].setVisible(has_selected_signal and has_selected_motor and has_data)

    def send_pid_once(self):
        for i in range(self.active_motor_count):
            self.send_set_pid(i, safe_float(self.entry_kp[i]), safe_float(self.entry_ki[i]), safe_float(self.entry_kd[i]))

    def start(self):
        self.running = True
        self.send_pid_speed()

    def stop(self):
        self.running = False
        self.send_frame(TYPE_PAUSE, struct.pack("<B", 1))

    def save_pid_to_flash(self):
        if not (self.ser and self.ser.is_open):
            self.set_status("保存PID失败: 串口未连接")
            return
        self.drain_pending_ack_events()
        self.save_transaction = {
            "next_motor": 0,
            "phase": "set",
            "deadline": 0.0,
            "retries": 0,
            "started_at": time.monotonic(),
        }
        self.send_next_save_pid()

    def drain_pending_ack_events(self):
        kept_events = []
        while True:
            try:
                event_type, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break
            if event_type != "ack":
                kept_events.append((event_type, payload))
        for event in kept_events:
            self.event_queue.put(event)

    def send_next_save_pid(self):
        if self.save_transaction is None:
            return
        motor_index = self.save_transaction["next_motor"]
        if motor_index >= self.active_motor_count:
            self.save_transaction["phase"] = "save"
            self.save_transaction["retries"] = 0
            self.send_current_save_pid_step()
            return
        self.save_transaction["phase"] = "set"
        self.save_transaction["retries"] = 0
        self.send_current_save_pid_step()

    def send_current_save_pid_step(self):
        if self.save_transaction is None:
            return
        phase = self.save_transaction["phase"]
        if phase == "save":
            self.send_frame(TYPE_SAVE_PID)
            self.save_transaction["deadline"] = time.monotonic() + SAVE_FLASH_ACK_TIMEOUT
            self.set_status("保存PID: 已发送保存命令")
            return

        motor_index = self.save_transaction["next_motor"]
        self.send_set_pid(
            motor_index,
            safe_float(self.entry_kp[motor_index]),
            safe_float(self.entry_ki[motor_index]),
            safe_float(self.entry_kd[motor_index]),
        )
        self.save_transaction["deadline"] = time.monotonic() + SAVE_SET_PID_ACK_TIMEOUT
        self.set_status(f"保存PID: 设置 M{motor_index}")

    def check_save_transaction_timeout(self):
        if self.save_transaction is None:
            return
        if time.monotonic() < self.save_transaction["deadline"]:
            return
        if self.save_transaction["retries"] >= SAVE_MAX_RETRIES:
            phase = self.save_transaction["phase"]
            if phase == "save":
                self.set_status("保存PID失败: 保存命令ACK超时")
            else:
                self.set_status(f"保存PID失败: M{self.save_transaction['next_motor']} SET_PID ACK超时")
            self.save_transaction = None
            return
        self.save_transaction["retries"] += 1
        self.send_current_save_pid_step()

    def load_pid_from_flash(self):
        self.send_frame(TYPE_LOAD_PID)

    def read_pid_from_controller(self):
        self.send_frame(TYPE_READ_PID)

    def load_default_pid(self):
        self.send_frame(TYPE_DEFAULT_PID)

    def send_pid_speed(self):
        if not self.running:
            return
        if self.save_transaction is not None:
            return
        self.send_pid_once()
        self.send_frame(TYPE_SET_SPEED, struct.pack("<f", safe_float(self.entry_speed)))
        self.send_frame(TYPE_PAUSE, struct.pack("<B", 0))

    def serial_reader(self):
        while self.reader_running:
            try:
                if self.ser and self.ser.is_open:
                    waiting = self.ser.in_waiting
                    read_size = min(max(waiting, 1), 512)
                    chunk = self.ser.read(read_size)
                    if chunk:
                        self.rx_bytes += len(chunk)
                        self.rx_buffer.extend(chunk)
                        for frame_type, payload in self.parse_frames(max_frames=16):
                            self.rx_frames += 1
                            self.handle_frame(frame_type, payload)
                        if len(self.rx_buffer) > 4096:
                            del self.rx_buffer[: len(self.rx_buffer) - 4096]
                        self.rx_short_yields += 1
                        time.sleep(0)
                else:
                    time.sleep(0.02)
            except Exception as exc:
                self.event_queue.put(("status", f"串口读取错误: {exc}"))
                break

    def parse_frames(self, max_frames=None):
        frames = []
        while True:
            if max_frames is not None and len(frames) >= max_frames:
                return frames
            header_index = self.rx_buffer.find(bytes([FRAME_HEADER]))
            if header_index < 0:
                self.rx_buffer.clear()
                return frames
            if header_index > 0:
                del self.rx_buffer[:header_index]
            if len(self.rx_buffer) < 3:
                return frames
            length = self.rx_buffer[1]
            if length < 5:
                del self.rx_buffer[0]
                continue
            if len(self.rx_buffer) < length:
                return frames
            frame = bytes(self.rx_buffer[:length])
            del self.rx_buffer[:length]
            received_crc = struct.unpack_from("<H", frame, length - 2)[0]
            if crc16(frame[:-2]) != received_crc:
                self.rx_crc_errors += 1
                continue
            frames.append((frame[2], frame[3:-2]))

    def handle_frame(self, frame_type: int, payload: bytes):
        if frame_type == TYPE_ACK:
            self.handle_ack(payload)
        elif frame_type == TYPE_PID_PARAMS:
            self.handle_pid_params(payload)
        elif frame_type == TYPE_MOTOR_TELEMETRY:
            self.handle_telemetry(payload)

    def handle_ack(self, payload: bytes):
        if len(payload) != 4:
            return
        command, status, motor_count, _ = struct.unpack("<BBBB", payload)
        self.active_motor_count = max(1, min(MAX_MOTORS, motor_count))
        self.event_queue.put(("status", f"ACK cmd=0x{command:02X} {'OK' if status == 0 else 'ERR'} motors={self.active_motor_count}"))
        self.event_queue.put(("ack", (time.monotonic(), command, status)))

    def handle_pid_params(self, payload: bytes):
        if len(payload) != 52:
            return
        count = payload[0]
        self.active_motor_count = max(1, min(MAX_MOTORS, count))
        params = []
        offset = 4
        for _ in range(MAX_MOTORS):
            params.append(struct.unpack_from("<fff", payload, offset))
            offset += 12
        self.event_queue.put(("pid_params", params))
        self.event_queue.put(("status", f"读取 PID: {self.active_motor_count} 路"))

    def handle_telemetry(self, payload: bytes):
        if len(payload) in {44, 48}:
            if len(payload) == 48:
                values = struct.unpack("<BBHIQfffffffi", payload)
            else:
                values = struct.unpack("<BBHIQffffffi", payload)
            signal_values = {}
            for signal_index, (name, _label) in enumerate(SIGNALS):
                if 5 + signal_index >= len(values) - 1:
                    continue
                signal_values[name] = values[5 + signal_index]
            now = time.monotonic()
            self.telemetry_queue.put((now, {
                "motor": values[0],
                "motor_count": values[1],
                "signal_mask": (1 << len(signal_values)) - 1,
                "sequence": values[3],
                "time_us": values[4],
                "signals": signal_values,
            }))
            return

        if len(payload) < 16:
            return
        motor, motor_count, signal_mask, sequence, device_time_us = struct.unpack_from("<BBHIQ", payload, 0)
        expected_len = 16 + 4 * signal_mask.bit_count()
        if len(payload) != expected_len:
            return
        signal_values = {}
        offset = 16
        for signal_index, (name, _label) in enumerate(SIGNALS):
            if (signal_mask & (1 << signal_index)) == 0:
                continue
            signal_values[name] = struct.unpack_from("<f", payload, offset)[0]
            offset += 4
        now = time.monotonic()
        values = {
            "motor": motor,
            "motor_count": motor_count,
            "signal_mask": signal_mask,
            "sequence": sequence,
            "time_us": device_time_us,
            "signals": signal_values,
        }
        self.telemetry_queue.put((now, values))

    def process_events(self):
        for _ in range(200):
            try:
                event_type, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break
            if event_type == "status":
                self.set_status(payload)
            elif event_type == "pid_params":
                self.apply_pid_params(payload)
            elif event_type == "ack":
                self.handle_ack_event(*payload)
        self.check_save_transaction_timeout()

    def set_status(self, text: str):
        self.status_label.setText(text)

    def apply_pid_params(self, params):
        for i, (kp, ki, kd) in enumerate(params[:MAX_MOTORS]):
            self.entry_kp[i].setText(f"{kp:.6g}")
            self.entry_ki[i].setText(f"{ki:.6g}")
            self.entry_kd[i].setText(f"{kd:.6g}")

    def handle_ack_event(self, received_at, command, status):
        if self.save_transaction is None:
            return
        if received_at < self.save_transaction["started_at"]:
            return
        expected_command = TYPE_SAVE_PID if self.save_transaction["phase"] == "save" else TYPE_SET_PID
        if command != expected_command:
            return
        if status != 0:
            self.set_status(f"保存PID失败: cmd=0x{command:02X}")
            self.save_transaction = None
            return
        if command == TYPE_SET_PID:
            self.save_transaction["next_motor"] += 1
            self.send_next_save_pid()
        elif command == TYPE_SAVE_PID:
            self.set_status("保存PID完成，正在读取校验")
            self.save_transaction = None
            self.send_frame(TYPE_READ_PID)

    def current_frame_rate(self):
        now = time.monotonic()
        while self.valid_frame_times and now - self.valid_frame_times[0] > 1.0:
            self.valid_frame_times.popleft()
        return len(self.valid_frame_times)

    def drain_telemetry_queue(self):
        drained = 0
        queue_depth = self.telemetry_queue.qsize()
        if queue_depth > self.telemetry_queue_max_depth:
            self.telemetry_queue_max_depth = queue_depth
        while True:
            try:
                now, values = self.telemetry_queue.get_nowait()
            except queue.Empty:
                break
            motor_index = values["motor"] if isinstance(values, dict) else values[0]
            if 0 <= motor_index < MAX_MOTORS:
                if isinstance(values, dict):
                    signal_values = values["signals"]
                    for name, value in signal_values.items():
                        if not self.signal_checks[name].isChecked():
                            continue
                        self.telemetry_times[name][motor_index].append(self.sample_index)
                        self.telemetry_values[name][motor_index].append(value)
                    self.latest_telemetry_sequence = values["sequence"]
                else:
                    for signal_index, (name, _label) in enumerate(SIGNALS):
                        if not self.signal_checks[name].isChecked():
                            continue
                        self.telemetry_times[name][motor_index].append(self.sample_index)
                        self.telemetry_values[name][motor_index].append(values[5 + signal_index])
                    self.latest_telemetry_sequence = values[3]
                self.latest_telemetry_time_us = now
                self.sample_index += 1
                self.telemetry_version += 1
                self.valid_frame_times.append(now)
                drained += 1
            else:
                self.invalid_motor_frames += 1
        return drained

    def update_plot(self):
        t0 = time.perf_counter()
        if self.last_plot_tick is not None:
            tick_gap_ms = (t0 - self.last_plot_tick) * 1000.0
            if tick_gap_ms > self.max_plot_tick_gap_ms:
                self.max_plot_tick_gap_ms = tick_gap_ms
        self.last_plot_tick = t0
        drained = self.drain_telemetry_queue()
        t_drain = time.perf_counter()
        latest_version = self.telemetry_version
        newest_sample_index = self.sample_index
        visible_times = {
            name: [list(times) for times in motor_times]
            for name, motor_times in self.telemetry_times.items()
            if self.signal_checks[name].isChecked()
        }
        visible_values = {
            name: [list(values) for values in motor_values]
            for name, motor_values in self.telemetry_values.items()
            if self.signal_checks[name].isChecked()
        }
        t_snapshot = time.perf_counter()
        if latest_version <= 0 or not visible_times:
            return

        x_span = max(50, self.x_points.value())
        if latest_version == self.last_plotted_version:
            return
        self.last_plot_version_delta = latest_version - self.last_plotted_version
        self.last_plotted_version = latest_version
        oldest_sample_index = newest_sample_index - x_span

        plot_y_ranges = [
            [math.inf, -math.inf]
            for _ in self.plots
        ]
        point_counts = []
        for motor_index in range(MAX_MOTORS):
            motor_visible = self.motor_checks[motor_index].isChecked()
            motor_point_count = 0
            for name, _label in SIGNALS:
                signal_times = visible_times.get(name)
                visible = motor_visible and signal_times is not None
                values = []
                x_values = []
                if visible:
                    times = signal_times[motor_index]
                    start_index = bisect.bisect_left(times, oldest_sample_index)
                    x_values = [sample_time - newest_sample_index for sample_time in times[start_index:]]
                    values = visible_values[name][motor_index][start_index:]
                    visible = bool(x_values)
                for curve_index, curve in enumerate(self.curves[(motor_index, name)]):
                    was_visible = self.curve_visible[(motor_index, name)][curve_index]
                    if was_visible != visible:
                        curve.setVisible(visible)
                        self.curve_visible[(motor_index, name)][curve_index] = visible
                    if visible:
                        curve.setData(x_values, values)
                    elif was_visible:
                        curve.setData([], [])
                if visible:
                    if len(x_values) > motor_point_count:
                        motor_point_count = len(x_values)
                    if values:
                        local_min = min(values)
                        local_max = max(values)
                        for plot_index, plot_info in enumerate(self.plots):
                            if name not in plot_info["signals"]:
                                continue
                            if local_min < plot_y_ranges[plot_index][0]:
                                plot_y_ranges[plot_index][0] = local_min
                            if local_max > plot_y_ranges[plot_index][1]:
                                plot_y_ranges[plot_index][1] = local_max
            if motor_visible and motor_point_count:
                point_counts.append(f"M{motor_index}:{motor_point_count}")

        t_curves = time.perf_counter()
        for plot_index, plot_info in enumerate(self.plots):
            has_data = plot_y_ranges[plot_index][0] != math.inf
            if plot_info["plot"].isVisible() != has_data:
                plot_info["plot"].setVisible(has_data)
            plot_info["plot"].setXRange(-x_span, 0, padding=0)
        if self.y_auto.isChecked():
            for plot_index, plot_info in enumerate(self.plots):
                y_min, y_max = plot_y_ranges[plot_index]
                if y_min != math.inf:
                    if y_min == y_max:
                        y_min -= 1.0
                        y_max += 1.0
                    padding = max((y_max - y_min) * 0.08, 1.0)
                    next_y_range = (y_min - padding, y_max + padding)
                    if self.should_update_y_range(plot_info["last_y_range"], next_y_range):
                        plot_info["plot"].setYRange(next_y_range[0], next_y_range[1], padding=0)
                        plot_info["last_y_range"] = next_y_range
        else:
            next_y_range = (self.y_min.value(), self.y_max.value())
            for plot_info in self.plots:
                if next_y_range != plot_info["last_y_range"]:
                    plot_info["plot"].setYRange(next_y_range[0], next_y_range[1], padding=0)
                    plot_info["last_y_range"] = next_y_range
        t_ranges = time.perf_counter()

        now = time.monotonic()
        self.plot_frame_times.append(now)
        while self.plot_frame_times and now - self.plot_frame_times[0] > 1.0:
            self.plot_frame_times.popleft()
        if now - self.last_status_update >= 0.2:
            self.last_status_update = now
            self.last_plot_profile = (
                f"入队 {(t_drain - t0) * 1000:.1f}ms/{drained} | "
                f"快照 {(t_snapshot - t0) * 1000:.1f}ms | "
                f"曲线 {(t_curves - t_snapshot) * 1000:.1f}ms | "
                f"坐标 {(t_ranges - t_curves) * 1000:.1f}ms | "
                f"总 {(t_ranges - t0) * 1000:.1f}ms"
            )
            self.set_status(
                f"采样 {self.current_frame_rate()} fps | 绘图更新 {len(self.plot_frame_times)} fps | "
                f"点数 {' '.join(point_counts)} | {self.last_plot_profile} | "
                f"timer最大间隔 {self.max_plot_tick_gap_ms:.0f}ms | "
                f"批量Δ{self.last_plot_version_delta} | "
                f"队列峰值{self.telemetry_queue_max_depth} | "
                f"RX {self.rx_frames}帧 {self.rx_bytes // 1024}KB CRC{self.rx_crc_errors} 无效M{self.invalid_motor_frames}"
            )
            self.max_plot_tick_gap_ms = 0.0
            self.telemetry_queue_max_depth = 0

    def should_update_y_range(self, last_y_range, next_y_range):
        if last_y_range is None:
            return True
        old_min, old_max = last_y_range
        new_min, new_max = next_y_range
        old_span = max(old_max - old_min, 1.0)
        return abs(new_min - old_min) > old_span * 0.03 or abs(new_max - old_max) > old_span * 0.03

    def closeEvent(self, event: QtGui.QCloseEvent):
        self.disconnect_serial()
        event.accept()


def main():
    pg.setConfigOptions(antialias=False, foreground="k")
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
