#!/usr/bin/env python3
"""
Rescue26 Serial Bridge — ROS 2 Node
=====================================
Reads the ESP32 binary protocol over serial and publishes sensor/encoder data
as standard ROS 2 topics.  Also subscribes to command topics so the rest of
the ROS graph can drive the robot.

Protocol frame: [0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC:1]
CRC = XOR of TYPE, LEN_H, LEN_L, and every payload byte.

Robot types
-----------
ROBOT_MAIN      — 2 traction encoders + 1 joined flipper encoder
ROBOT_SECONDARY — 2 traction encoders + 4 independent flipper encoders (FL/FR/RL/RR)

Set ROBOT_TYPE at the top of this file to match the connected hardware.
No type information is transmitted over the serial protocol; both sides must
agree on the payload layout beforehand.

Published topics
----------------
Common (both robots):
  /encoders/speed_left_rpm    std_msgs/Float32       left track speed (RPM)
  /encoders/speed_right_rpm   std_msgs/Float32       right track speed (RPM)
  /sensors/magnetometer       sensor_msgs/MagneticField
  /sensors/thermal            sensor_msgs/Image      32×24 float32 mono °C
  /sensors/gas/ppm_lpg        std_msgs/Float32
  /sensors/gas/ppm_co         std_msgs/Float32
  /sensors/gas/ppm_smoke      std_msgs/Float32
  /robot/status               std_msgs/String        JSON (mode, flags, uptime, PPM)

ROBOT_MAIN only:
  /encoders/flipper_angle_deg std_msgs/Float32       single joined flipper

ROBOT_SECONDARY only:
  /encoders/flipper_fl_deg    std_msgs/Float32       front-left  flipper
  /encoders/flipper_fr_deg    std_msgs/Float32       front-right flipper
  /encoders/flipper_rl_deg    std_msgs/Float32       rear-left   flipper
  /encoders/flipper_rr_deg    std_msgs/Float32       rear-right  flipper

Subscribed topics (commands to ESP32, both robots):
  /robot/sensor_enable        std_msgs/UInt8         bitmask bit0=mag bit1=thermal bit2=gas
  /robot/estop                std_msgs/Bool          True=ESTOP, False=ESTOP_CLEAR
  /robot/arm_joints           std_msgs/Float32MultiArray  6 joint angles in degrees

Parameters
----------
  port  string  default '/dev/ttyUSB0'
  baud  int     default 921600

Usage
-----
  ros2 run rescue26_bridge robot_serial_bridge --ros-args -p port:=/dev/ttyUSB0
"""

import json
import struct
import threading

import numpy as np
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, MagneticField
from std_msgs.msg import Bool, Float32, Float32MultiArray, String, UInt8

# ─── Robot type — edit this line to match the connected hardware ──────────────
# 'main'      → TelemetryPayload      (24 bytes): 2 tracks + 1 joined flipper
# 'secondary' → TelemetrySecPayload   (30 bytes): 2 tracks + 4 independent flippers
ROBOT_TYPE = 'main'

# ── Protocol constants ────────────────────────────────────────────────────────
SOF_0 = 0xAA
SOF_1 = 0x55

# ESP32 → PC
MSG_TELEMETRY      = 0x01
MSG_SENSOR_THERMAL = 0x02
MSG_SENSOR_MAG     = 0x03
MSG_SENSOR_GAS     = 0x04
MSG_STATUS         = 0x05

# PC → ESP32
MSG_ARM_JOINTS    = 0x10
MSG_SENSOR_ENABLE = 0x11
MSG_ESTOP         = 0x12
MSG_ESTOP_CLEAR   = 0x13

PPM_CHANNELS = 6
THERMAL_W    = 32
THERMAL_H    = 24
MAX_PAYLOAD  = 1600

# ── Struct formats (little-endian, #pragma pack(1) on ESP32 side) ─────────────
#
# TelemetryPayload (ROBOT_MAIN, 24 bytes):
#   mode(u8) flags(u8) ppm[6](u16×6) speed_l(i16) speed_r(i16) flipper(i16) uptime(u32)
FMT_TELEM_MAIN = '<BB6HhhhI'

# TelemetrySecPayload (ROBOT_SECONDARY, 30 bytes):
#   mode(u8) flags(u8) ppm[6](u16×6) speed_l(i16) speed_r(i16)
#   fl(i16) fr(i16) rl(i16) rr(i16) uptime(u32)
FMT_TELEM_SEC = '<BB6HhhhhhhI'

# MagPayload (8 bytes): x_uT100 y_uT100 z_uT100 heading_deg10  (all i16)
FMT_MAG = '<hhhh'

# GasPayload (8 bytes): rs_ro_100 ppm_lpg ppm_co ppm_smoke  (all i16)
FMT_GAS = '<hhhh'

# ThermalPayload (1538 bytes): pixels[768](i16) ambient_C10(i16)
FMT_THERMAL = f'<{THERMAL_W * THERMAL_H + 1}h'

# Expected payload sizes — used to validate frames before unpacking
_TELEM_SIZE = {
    'main':      struct.calcsize(FMT_TELEM_MAIN),   # 24
    'secondary': struct.calcsize(FMT_TELEM_SEC),    # 30
}


# ── ROS 2 node ────────────────────────────────────────────────────────────────
class RobotSerialBridge(Node):

    def __init__(self):
        super().__init__('robot_serial_bridge')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.get_logger().info(f'Robot type: {ROBOT_TYPE}')

        # ── Publishers — common to both robots ───────────────────────────────
        self.pub_speed_left  = self.create_publisher(Float32, '/encoders/speed_left_rpm',  10)
        self.pub_speed_right = self.create_publisher(Float32, '/encoders/speed_right_rpm', 10)
        self.pub_magnetometer = self.create_publisher(MagneticField, '/sensors/magnetometer', 10)
        self.pub_thermal      = self.create_publisher(Image, '/sensors/thermal', 10)
        self.pub_gas_lpg      = self.create_publisher(Float32, '/sensors/gas/ppm_lpg',   10)
        self.pub_gas_co       = self.create_publisher(Float32, '/sensors/gas/ppm_co',    10)
        self.pub_gas_smoke    = self.create_publisher(Float32, '/sensors/gas/ppm_smoke', 10)
        self.pub_status       = self.create_publisher(String, '/robot/status', 10)

        # ── Publishers — robot-type-specific flipper topics ───────────────────
        if ROBOT_TYPE == 'main':
            # Single joined flipper driven from one PCNT encoder
            self.pub_flipper = self.create_publisher(
                Float32, '/encoders/flipper_angle_deg', 10)
        else:
            # Four independent flippers driven by VESC CAN controllers
            self.pub_flipper_fl = self.create_publisher(Float32, '/encoders/flipper_fl_deg', 10)
            self.pub_flipper_fr = self.create_publisher(Float32, '/encoders/flipper_fr_deg', 10)
            self.pub_flipper_rl = self.create_publisher(Float32, '/encoders/flipper_rl_deg', 10)
            self.pub_flipper_rr = self.create_publisher(Float32, '/encoders/flipper_rr_deg', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(UInt8,             '/robot/sensor_enable', self._on_sensor_enable, 10)
        self.create_subscription(Bool,              '/robot/estop',         self._on_estop,         10)
        self.create_subscription(Float32MultiArray, '/robot/arm_joints',    self._on_arm_joints,    10)

        # ── Serial port ──────────────────────────────────────────────────────
        try:
            self._serial = serial.Serial(port, baud, timeout=1.0)
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            raise

        self._serial_lock = threading.Lock()

        # ── Latest values for the monitor timer ──────────────────────────────
        self._latest = {}

        # ── 1 Hz monitor timer ───────────────────────────────────────────────
        self.create_timer(1.0, self._print_monitor)

        # ── RX thread ────────────────────────────────────────────────────────
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name='serial_rx')
        self._rx_thread.start()

        self.get_logger().info(f'Serial bridge started — {port} @ {baud} baud')

    def destroy_node(self):
        self._running = False
        self._serial.close()
        super().destroy_node()

    # ── CRC ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _crc(msg_type: int, payload: bytes) -> int:
        length = len(payload)
        acc = msg_type ^ (length >> 8) ^ (length & 0xFF)
        for b in payload:
            acc ^= b
        return acc & 0xFF

    # ── TX ───────────────────────────────────────────────────────────────────
    def _send_frame(self, msg_type: int, payload: bytes) -> None:
        length = len(payload)
        header = bytes([SOF_0, SOF_1, msg_type, length >> 8, length & 0xFF])
        frame  = header + payload + bytes([self._crc(msg_type, payload)])
        with self._serial_lock:
            self._serial.write(frame)

    # ── RX loop (background thread) ──────────────────────────────────────────
    def _rx_loop(self) -> None:
        (S_SOF0, S_SOF1, S_TYPE, S_LEN_H, S_LEN_L, S_PAYLOAD, S_CRC) = range(7)

        state    = S_SOF0
        msg_type = 0
        length   = 0
        idx      = 0
        buf      = bytearray(MAX_PAYLOAD)
        crc_acc  = 0

        while self._running:
            try:
                raw = self._serial.read(1)
            except serial.SerialException as e:
                if self._running:
                    self.get_logger().warn(f'Serial read error: {e}')
                break

            if not raw:
                continue

            b = raw[0]

            if state == S_SOF0:
                if b == SOF_0:
                    state = S_SOF1

            elif state == S_SOF1:
                state = S_TYPE if b == SOF_1 else S_SOF0

            elif state == S_TYPE:
                msg_type = b
                crc_acc  = b
                state    = S_LEN_H

            elif state == S_LEN_H:
                length   = b << 8
                crc_acc ^= b
                state    = S_LEN_L

            elif state == S_LEN_L:
                length  |= b
                crc_acc ^= b
                idx      = 0
                if length == 0:
                    state = S_CRC
                elif length > MAX_PAYLOAD:
                    self.get_logger().warn(f'Oversized frame ({length} B) — discarding')
                    state = S_SOF0
                else:
                    state = S_PAYLOAD

            elif state == S_PAYLOAD:
                buf[idx]  = b
                crc_acc  ^= b
                idx       += 1
                if idx >= length:
                    state = S_CRC

            elif state == S_CRC:
                if b == crc_acc:
                    self._dispatch(msg_type, bytes(buf[:length]))
                else:
                    self.get_logger().debug(
                        f'CRC mismatch type=0x{msg_type:02X} '
                        f'got=0x{b:02X} expected=0x{crc_acc:02X}')
                state = S_SOF0

    # ── 1 Hz terminal monitor ────────────────────────────────────────────────
    def _print_monitor(self) -> None:
        d = self._latest
        if not d:
            print('[robot_serial_bridge] waiting for data...')
            return

        lines = ['─' * 52]

        if ROBOT_TYPE == 'main':
            lines.append(f'  Encoders  │ L: {d.get("speed_l", 0.0):7.2f} RPM'
                         f'  R: {d.get("speed_r", 0.0):7.2f} RPM')
            lines.append(f'            │ Flipper: {d.get("flipper", 0.0):6.1f} °')
        else:
            lines.append(f'  Encoders  │ L: {d.get("speed_l", 0.0):7.2f} RPM'
                         f'  R: {d.get("speed_r", 0.0):7.2f} RPM')
            lines.append(f'  Flippers  │ FL:{d.get("fl", 0.0):6.1f}°'
                         f'  FR:{d.get("fr", 0.0):6.1f}°'
                         f'  RL:{d.get("rl", 0.0):6.1f}°'
                         f'  RR:{d.get("rr", 0.0):6.1f}°')

        if 'mag' in d:
            mx, my, mz = d['mag']
            lines.append(f'  Mag (µT)  │ X:{mx:8.2f}  Y:{my:8.2f}  Z:{mz:8.2f}')

        if 'gas' in d:
            lpg, co, smoke = d['gas']
            lines.append(f'  Gas (ppm) │ LPG:{lpg:6.0f}  CO:{co:6.0f}  Smoke:{smoke:6.0f}')

        if 'thermal' in d:
            lines.append(f'  Thermal   │ min:{d["thermal"][0]:5.1f}°C'
                         f'  max:{d["thermal"][1]:5.1f}°C'
                         f'  amb:{d["thermal"][2]:5.1f}°C')

        if 'mode' in d:
            mode_names = {0:'INIT',1:'STANDBY',2:'NORMAL',3:'ARM',4:'ESTOP',5:'FLIPPER'}
            lines.append(f'  Status    │ mode={mode_names.get(d["mode"], d["mode"])}'
                         f'  uptime={d.get("uptime_ms", 0) / 1000.0:.1f}s'
                         f'  estop={d.get("estop", False)}')

        lines.append('─' * 52)
        print('\n'.join(lines))

    # ── Frame dispatch ────────────────────────────────────────────────────────
    def _dispatch(self, msg_type: int, payload: bytes) -> None:
        try:
            if   msg_type == MSG_TELEMETRY:      self._handle_telemetry(payload)
            elif msg_type == MSG_SENSOR_MAG:     self._handle_mag(payload)
            elif msg_type == MSG_SENSOR_GAS:     self._handle_gas(payload)
            elif msg_type == MSG_SENSOR_THERMAL: self._handle_thermal(payload)
            elif msg_type == MSG_STATUS:         self._handle_status(payload)
        except Exception as e:
            self.get_logger().warn(f'Error handling type=0x{msg_type:02X}: {e}')

    # ── Telemetry (0x01) ─────────────────────────────────────────────────────
    def _handle_telemetry(self, payload: bytes) -> None:
        if ROBOT_TYPE == 'main':
            self._handle_telemetry_main(payload)
        else:
            self._handle_telemetry_secondary(payload)

    def _handle_telemetry_main(self, payload: bytes) -> None:
        # TelemetryPayload (firmware ROBOT_MAIN): 24 bytes
        # mode(u8) flags(u8) ppm[6](u16×6) speed_l(i16) speed_r(i16) flipper(i16) uptime(u32)
        if len(payload) < _TELEM_SIZE['main']:
            return
        vals    = struct.unpack_from(FMT_TELEM_MAIN, payload)
        mode    = vals[0]
        flags   = vals[1]
        ppm     = list(vals[2:8])
        speed_l = vals[8]   # RPM × 10
        speed_r = vals[9]
        flipper = vals[10]  # degrees × 10
        uptime  = vals[11]

        msg = Float32()
        msg.data = speed_l / 10.0;  self.pub_speed_left.publish(msg)
        msg.data = speed_r / 10.0;  self.pub_speed_right.publish(msg)
        msg.data = flipper / 10.0;  self.pub_flipper.publish(msg)

        self._latest.update({'speed_l': speed_l / 10.0, 'speed_r': speed_r / 10.0,
                             'flipper': flipper / 10.0})
        self._publish_status(mode, flags, uptime, ppm)

    def _handle_telemetry_secondary(self, payload: bytes) -> None:
        # TelemetrySecPayload (firmware ROBOT_SECONDARY): 30 bytes
        # mode(u8) flags(u8) ppm[6](u16×6) speed_l(i16) speed_r(i16)
        # fl(i16) fr(i16) rl(i16) rr(i16) uptime(u32)
        if len(payload) < _TELEM_SIZE['secondary']:
            return
        vals    = struct.unpack_from(FMT_TELEM_SEC, payload)
        mode    = vals[0]
        flags   = vals[1]
        ppm     = list(vals[2:8])
        speed_l = vals[8]   # RPM × 10
        speed_r = vals[9]
        fl      = vals[10]  # degrees × 10
        fr      = vals[11]
        rl      = vals[12]
        rr      = vals[13]
        uptime  = vals[14]

        msg = Float32()
        msg.data = speed_l / 10.0;  self.pub_speed_left.publish(msg)
        msg.data = speed_r / 10.0;  self.pub_speed_right.publish(msg)
        msg.data = fl / 10.0;       self.pub_flipper_fl.publish(msg)
        msg.data = fr / 10.0;       self.pub_flipper_fr.publish(msg)
        msg.data = rl / 10.0;       self.pub_flipper_rl.publish(msg)
        msg.data = rr / 10.0;       self.pub_flipper_rr.publish(msg)

        self._latest.update({'speed_l': speed_l / 10.0, 'speed_r': speed_r / 10.0,
                             'fl': fl / 10.0, 'fr': fr / 10.0,
                             'rl': rl / 10.0, 'rr': rr / 10.0})
        self._publish_status(mode, flags, uptime, ppm)

    def _publish_status(self, mode: int, flags: int, uptime: int, ppm: list) -> None:
        self._latest.update({'mode': mode, 'estop': bool(flags & 0x08), 'uptime_ms': uptime})
        msg = String()
        msg.data = json.dumps({
            'mode':      mode,
            'ppm_ok':    bool(flags & 0x01),
            'sensors':   bool(flags & 0x02),
            'can_ok':    bool(flags & 0x04),
            'estop':     bool(flags & 0x08),
            'uptime_ms': uptime,
            'ppm':       ppm,
        })
        self.pub_status.publish(msg)

    # ── Magnetometer (0x03) ───────────────────────────────────────────────────
    def _handle_mag(self, payload: bytes) -> None:
        if len(payload) < struct.calcsize(FMT_MAG):
            return
        x100, y100, z100 = struct.unpack_from('<hhh', payload)

        msg = MagneticField()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # µT → Tesla (1 µT = 1e-6 T)
        msg.magnetic_field.x = (x100 / 100.0) * 1e-6
        msg.magnetic_field.y = (y100 / 100.0) * 1e-6
        msg.magnetic_field.z = (z100 / 100.0) * 1e-6
        msg.magnetic_field_covariance = [0.0] * 9
        self.pub_magnetometer.publish(msg)
        self._latest['mag'] = (x100 / 100.0, y100 / 100.0, z100 / 100.0)

    # ── Gas (0x04) ────────────────────────────────────────────────────────────
    def _handle_gas(self, payload: bytes) -> None:
        if len(payload) < struct.calcsize(FMT_GAS):
            return
        _rs_ro100, lpg, co, smoke = struct.unpack_from(FMT_GAS, payload)

        msg = Float32()
        msg.data = float(lpg);   self.pub_gas_lpg.publish(msg)
        msg.data = float(co);    self.pub_gas_co.publish(msg)
        msg.data = float(smoke); self.pub_gas_smoke.publish(msg)

    # ── Thermal (0x02) ────────────────────────────────────────────────────────
    def _handle_thermal(self, payload: bytes) -> None:
        if len(payload) < struct.calcsize(FMT_THERMAL):
            return
        n    = THERMAL_W * THERMAL_H
        vals = struct.unpack_from(FMT_THERMAL, payload)

        pixels = np.array(vals[:n], dtype=np.float32) / 10.0  # °C×10 → °C

        img              = Image()
        img.header.stamp    = self.get_clock().now().to_msg()
        img.header.frame_id = 'thermal_camera'
        img.height       = THERMAL_H
        img.width        = THERMAL_W
        img.encoding     = '32FC1'
        img.is_bigendian = False
        img.step         = THERMAL_W * 4
        img.data         = pixels.tobytes()
        self.pub_thermal.publish(img)

    # ── Status (0x05) ─────────────────────────────────────────────────────────
    def _handle_status(self, payload: bytes) -> None:
        if len(payload) < 3:
            return
        msg = String()
        msg.data = json.dumps({
            'type':        'status',
            'mode':        payload[0],
            'ppm_ok':      bool(payload[1] & 0x01),
            'minipc_ok':   bool(payload[1] & 0x02),
            'can_ok':      bool(payload[1] & 0x04),
            'estop':       bool(payload[1] & 0x08),
            'sensor_mask': payload[2],
        })
        self.pub_status.publish(msg)

    # ── Command subscribers ───────────────────────────────────────────────────
    def _on_sensor_enable(self, msg: UInt8) -> None:
        self._send_frame(MSG_SENSOR_ENABLE, bytes([msg.data & 0xFF]))

    def _on_estop(self, msg: Bool) -> None:
        self._send_frame(MSG_ESTOP if msg.data else MSG_ESTOP_CLEAR, b'')

    def _on_arm_joints(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 6:
            self.get_logger().warn('/robot/arm_joints needs 6 values')
            return
        payload = struct.pack('<6h', *(int(a * 100.0) for a in msg.data[:6]))
        self._send_frame(MSG_ARM_JOINTS, payload)


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = RobotSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
