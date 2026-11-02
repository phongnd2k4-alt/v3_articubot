#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 UART bridge for the 4-wheel AGV STM32F407 firmware.

Subscribes:
  /cmd_vel                  geometry_msgs/msg/Twist

Publishes:
  /agv/encoder_ticks        std_msgs/msg/Int32MultiArray (LF, LR, RF, RR)
  /agv/encoder_tick_rate    std_msgs/msg/Float32MultiArray (ticks/s, LF, LR, RF, RR)
  /agv/status               std_msgs/msg/String

Service:
  /agv/enable               std_srvs/srv/SetBool

Wire protocol retained from the working standalone Pi script:
  Header AA 55, length, message ID, sequence, payload, CRC16-CCITT-FALSE LE.
"""

import struct
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from std_srvs.srv import SetBool

try:
    import serial
except ImportError as exc:  # pragma: no cover - installation hint for the target Pi
    raise RuntimeError(
        "Missing pyserial. Install it with: sudo apt install python3-serial"
    ) from exc


HEADER = b"\xAA\x55"
ID_CMD_VEL = 0x01
ID_SET_MODE = 0x02
ID_PING = 0x04
ID_ODOM = 0x81
ID_STATUS = 0x84
ID_PONG = 0x85
WHEEL_NAMES = ("LF", "LR", "RF", "RR")


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_id: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > 255:
        raise ValueError("UART payload exceeds 255 bytes")
    body = bytes((len(payload), msg_id & 0xFF, seq & 0xFF)) + payload
    return HEADER + body + struct.pack("<H", crc16_ccitt_false(body))


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer[:] = (
                    self.buffer[-1:] if self.buffer[-1:] == HEADER[:1] else b""
                )
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                break
            payload_len = self.buffer[2]
            # Header(2) + length(1) + ID(1) + seq(1) + payload + CRC(2)
            total_len = payload_len + 7
            if len(self.buffer) < total_len:
                break
            frame = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]
            body = frame[2:-2]
            rx_crc = struct.unpack("<H", frame[-2:])[0]
            if crc16_ccitt_false(body) != rx_crc:
                continue
            msg_id = frame[3]
            seq = frame[4]
            payload = frame[5:-2]
            frames.append((msg_id, seq, payload))
        return frames


class AgvUartNode(Node):
    def __init__(self):
        super().__init__("agv_uart_node")

        self.declare_parameter("port", "/dev/serial0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("tx_period", 0.10)
        self.declare_parameter("max_linear_mm_s", 100)
        self.declare_parameter("max_angular_mrad_s", 400)
        self.declare_parameter("ping_period", 1.0)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.tx_period = float(self.get_parameter("tx_period").value)
        self.max_linear = abs(int(self.get_parameter("max_linear_mm_s").value))
        self.max_angular = abs(int(self.get_parameter("max_angular_mrad_s").value))
        self.ping_period = float(self.get_parameter("ping_period").value)

        self.serial_port = None
        self.parser = FrameParser()
        self.seq = 0
        self.enabled = False
        self.v_mm_s = 0
        self.w_mrad_s = 0
        self.last_cmd_time = 0.0
        self.last_tx_time = 0.0
        self.last_ping_time = 0.0
        self.last_open_attempt = 0.0
        self.previous_ticks = None
        self.previous_stm32_time = None

        self.encoder_pub = self.create_publisher(Int32MultiArray, "/agv/encoder_ticks", 10)
        self.rate_pub = self.create_publisher(Float32MultiArray, "/agv/encoder_tick_rate", 10)
        self.status_pub = self.create_publisher(String, "/agv/status", 10)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.enable_srv = self.create_service(SetBool, "/agv/enable", self.enable_callback)

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.get_logger().info(
            f"AGV UART bridge started: port={self.port}, baud={self.baud}, "
            "protocol=AA55/CRC16. Motor starts DISABLED."
        )
        self._try_open_serial()

    def _try_open_serial(self):
        now = time.monotonic()
        if self.serial_port is not None or now - self.last_open_attempt < 2.0:
            return
        self.last_open_attempt = now
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=0.1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            self.parser = FrameParser()
            self.get_logger().info(f"Opened UART {self.port} at {self.baud} baud (8N1).")
            # Fail-safe: start with motors disabled and zero velocity.
            self._send_velocity(0, 0)
            self._send_mode(0)
        except (serial.SerialException, OSError) as exc:
            self.serial_port = None
            self.get_logger().error(
                f"Cannot open {self.port}: {exc}. Check UART permissions, port and wiring. "
                "Will retry."
            )

    def _close_serial(self, reason: str):
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.enabled = False
        self.v_mm_s = 0
        self.w_mrad_s = 0
        self.get_logger().error(f"UART disconnected: {reason}. Motor state set to DISABLED.")

    def _send(self, msg_id: int, payload: bytes = b""):
        if self.serial_port is None:
            return False
        try:
            packet = build_frame(msg_id, self.seq, payload)
            self.serial_port.write(packet)
            self.seq = (self.seq + 1) & 0xFF
            return True
        except (serial.SerialException, OSError) as exc:
            self._close_serial(str(exc))
            return False

    def _send_velocity(self, v_mm_s: int, w_mrad_s: int):
        v = max(-32768, min(32767, int(v_mm_s)))
        w = max(-32768, min(32767, int(w_mrad_s)))
        return self._send(ID_CMD_VEL, struct.pack("<hh", v, w))

    def _send_mode(self, mode: int):
        return self._send(ID_SET_MODE, struct.pack("<B", 1 if mode else 0))

    def cmd_vel_callback(self, msg: Twist):
        # ROS units: m/s and rad/s. Firmware units: mm/s and mrad/s.
        v = int(round(msg.linear.x * 1000.0))
        w = int(round(msg.angular.z * 1000.0))
        self.v_mm_s = max(-self.max_linear, min(self.max_linear, v))
        self.w_mrad_s = max(-self.max_angular, min(self.max_angular, w))
        self.last_cmd_time = time.monotonic()

    def enable_callback(self, request, response):
        if self.serial_port is None:
            response.success = False
            response.message = f"UART {self.port} is not connected/open."
            return response
        if request.data:
            # Keep zero velocity while enabling. A fresh /cmd_vel is still required.
            self.v_mm_s = 0
            self.w_mrad_s = 0
            self._send_velocity(0, 0)
            ok = self._send_mode(1)
            self.enabled = bool(ok)
            response.success = bool(ok)
            response.message = "Motor ENABLE requested." if ok else "Failed to send ENABLE."
            if ok:
                self.get_logger().warning("Motor ENABLED. Ensure the AGV is safely lifted for first test.")
        else:
            self.v_mm_s = 0
            self.w_mrad_s = 0
            self._send_velocity(0, 0)
            ok = self._send_mode(0)
            self.enabled = False
            response.success = bool(ok)
            response.message = "Motor DISABLED." if ok else "UART unavailable; local state set DISABLED."
            self.get_logger().info("Motor DISABLED; zero velocity sent.")
        return response

    def _handle_frame(self, msg_id: int, payload: bytes):
        if msg_id == ID_ODOM and len(payload) == 20:
            stm32_ms = struct.unpack_from("<I", payload, 0)[0]
            ticks = struct.unpack_from("<4i", payload, 4)
            tick_rates = [0.0, 0.0, 0.0, 0.0]
            if self.previous_ticks is not None and self.previous_stm32_time is not None:
                dt_ms = (stm32_ms - self.previous_stm32_time) & 0xFFFFFFFF
                if dt_ms > 0:
                    tick_rates = [
                        (new - old) * 1000.0 / dt_ms
                        for old, new in zip(self.previous_ticks, ticks)
                    ]
            self.previous_ticks = ticks
            self.previous_stm32_time = stm32_ms

            ticks_msg = Int32MultiArray()
            ticks_msg.data = list(ticks)
            self.encoder_pub.publish(ticks_msg)

            rate_msg = Float32MultiArray()
            rate_msg.data = tick_rates
            self.rate_pub.publish(rate_msg)

        elif msg_id == ID_STATUS and len(payload) == 4:
            flags, mode, rx_errors = struct.unpack("<BBH", payload)
            status = String()
            status.data = f"mode={mode} flags=0x{flags:02X} rx_errors={rx_errors}"
            self.status_pub.publish(status)

        elif msg_id == ID_PONG and len(payload) == 4:
            # PONG confirms that the STM32 is receiving PING frames.
            pass

    def timer_callback(self):
        if self.serial_port is None:
            self._try_open_serial()
            return

        now = time.monotonic()

        # Non-blocking UART receive.
        try:
            data = self.serial_port.read(256)
            if data:
                for msg_id, _rx_seq, payload in self.parser.feed(data):
                    self._handle_frame(msg_id, payload)
        except (serial.SerialException, OSError) as exc:
            self._close_serial(str(exc))
            return

        # Transmit command regularly to satisfy the STM32 command timeout.
        if now - self.last_tx_time >= self.tx_period:
            cmd_is_fresh = self.last_cmd_time > 0.0 and (now - self.last_cmd_time) <= self.cmd_vel_timeout
            if self.enabled and cmd_is_fresh:
                self._send_velocity(self.v_mm_s, self.w_mrad_s)
            else:
                self._send_velocity(0, 0)
            self.last_tx_time = now

        # Periodic PING does not enable the motors.
        if now - self.last_ping_time >= self.ping_period:
            self._send(ID_PING, struct.pack("<I", 0x12345678))
            self.last_ping_time = now

    def destroy_node(self):
        # Best-effort stop and disable before closing UART.
        if self.serial_port is not None:
            try:
                self._send_velocity(0, 0)
                time.sleep(0.03)
                self._send_mode(0)
                time.sleep(0.03)
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AgvUartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
