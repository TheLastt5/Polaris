#!/usr/bin/env python3
# ===========================================================================
# stm32_bridge_node.py  –  STM32 ↔ ROS2 Seri Köprüsü
# Hedef: ROS2 Humble / Iron  |  Python 3.10+
#
# ROS_Frame_t beklenen seri format (little-endian, 35 byte):
#   '<I f f f f f f f B H'
#    ^header  ^7xfloat  ^fix  ^checksum
# ===========================================================================
import struct
import math
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from std_msgs.msg    import Float32

# ---------------------------------------------------------------------------
# Çerçeve formatı – robot_telemetry.h ROS_Frame_t ile birebir eşleşmeli
# ---------------------------------------------------------------------------
FRAME_FORMAT = '<IfffffffBH'
FRAME_SIZE   = struct.calcsize(FRAME_FORMAT)   # 35 byte
FRAME_HEADER = 0xAA55AA55

# ---------------------------------------------------------------------------
# IMU ölçüm belirsizliği (rad²) – EKF için gerekli
# ---------------------------------------------------------------------------
ORIENT_COV_ROLL  = 0.01   # ~5.7°
ORIENT_COV_PITCH = 0.01
ORIENT_COV_YAW   = 0.04   # yaw daha gürültülü


class STM32BridgeNode(Node):
    """
    STM32 seri çıkışını ROS2 topic'lerine dönüştürür.
    """

    def __init__(self):
        super().__init__('stm32_bridge')

        self.declare_parameter('port',     '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port     = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f'Seri port açıldı: {port} @ {baudrate}')
        except serial.SerialException as exc:
            self.get_logger().fatal(f'Seri port açılamadı: {exc}')
            raise SystemExit(1) from exc

        self.pub_imu     = self.create_publisher(Imu,       '/stm32/imu',         10)
        self.pub_gps     = self.create_publisher(NavSatFix, '/stm32/gps',         10)
        self.pub_enc_vel = self.create_publisher(Float32,   '/stm32/encoder_vel', 10)

        self.create_timer(1.0 / 50.0, self._read_frame)
        self._buf = b''

    @staticmethod
    def _euler_to_quat(roll: float, pitch: float, yaw: float):
        cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def _read_frame(self):
        try:
            self._buf += self.ser.read(FRAME_SIZE * 2)
        except serial.SerialException as exc:
            self.get_logger().warn(f'Seri okuma hatası: {exc}')
            return

        header_bytes = struct.pack('<I', FRAME_HEADER)
        idx = self._buf.find(header_bytes)

        if idx == -1:
            self._buf = self._buf[-3:]
            return

        self._buf = self._buf[idx:]

        if len(self._buf) < FRAME_SIZE:
            return

        raw        = self._buf[:FRAME_SIZE]
        self._buf  = self._buf[FRAME_SIZE:]

        try:
            (header,
             roll, pitch, yaw,
             enc_vel_left, enc_vel_right,
             lat, lon, fix,
             _csum) = struct.unpack(FRAME_FORMAT, raw)
        except struct.error as exc:
            self.get_logger().warn(f'Paket çözme hatası: {exc}')
            return

        now = self.get_clock().now().to_msg()
        self._publish_imu(now, roll, pitch, yaw)
        self._publish_gps(now, lat, lon, fix)
        self._publish_enc_vel(enc_vel_left, enc_vel_right)

    def _publish_imu(self, stamp, roll_deg, pitch_deg, yaw_deg):
        qx, qy, qz, qw = self._euler_to_quat(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg))

        msg = Imu()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'base_link'

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.orientation_covariance[0] = ORIENT_COV_ROLL
        msg.orientation_covariance[4] = ORIENT_COV_PITCH
        msg.orientation_covariance[8] = ORIENT_COV_YAW

        msg.angular_velocity_covariance[0]    = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(msg)

    def _publish_gps(self, stamp, lat, lon, fix):
        msg = NavSatFix()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'gps_link'
        msg.latitude        = float(lat)
        msg.longitude       = float(lon)
        msg.altitude        = 0.0

        msg.status.status  = NavSatStatus.STATUS_FIX if fix > 0 \
                              else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.position_covariance[0] = 9.0
        msg.position_covariance[4] = 9.0
        msg.position_covariance[8] = 25.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub_gps.publish(msg)

    def _publish_enc_vel(self, left: float, right: float):
        msg      = Float32()
        msg.data = (left + right) / 2.0
        self.pub_enc_vel.publish(msg)

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
