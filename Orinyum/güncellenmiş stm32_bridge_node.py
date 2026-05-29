#!/usr/bin/env python3
import struct
import serial
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32

FRAME_FORMAT = '<IfffffffBH'
FRAME_SIZE   = struct.calcsize(FRAME_FORMAT)
FRAME_HEADER = 0xAA55AA55

class STM32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        self.pub_imu     = self.create_publisher(Imu,       '/stm32/imu',         10)
        self.pub_gps     = self.create_publisher(NavSatFix, '/stm32/gps',         10)
        self.pub_enc_vel = self.create_publisher(Float32,   '/stm32/encoder_vel', 10)

        self.create_timer(0.020, self.read_frame)
        self.buf = b''

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = (math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
            - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2))
        qy = (math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
            + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2))
        qz = (math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
            - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2))
        qw = (math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
            + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2))
        return qx, qy, qz, qw

    def read_frame(self):
        self.buf += self.ser.read(FRAME_SIZE * 2)

        header_bytes = struct.pack('<I', FRAME_HEADER)
        idx = self.buf.find(header_bytes)
        if idx == -1:
            self.buf = b''
            return
        self.buf = self.buf[idx:]
        if len(self.buf) < FRAME_SIZE:
            return

        raw      = self.buf[:FRAME_SIZE]
        self.buf = self.buf[FRAME_SIZE:]

        (header, roll, pitch, yaw,
         enc_vel_left, enc_vel_right,
         lat, lon, fix, csum) = struct.unpack(FRAME_FORMAT, raw)

        now = self.get_clock().now().to_msg()

        # IMU mesajı — Euler → Quaternion
        imu_msg = Imu()
        imu_msg.header.stamp    = now
        imu_msg.header.frame_id = 'base_link'

        qx, qy, qz, qw = self.euler_to_quaternion(
            math.radians(roll),
            math.radians(pitch),
            math.radians(yaw))

        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        # EKF'in bu veriyi kullanabilmesi için covariance doldurulmalı
        imu_msg.orientation_covariance[0] = 0.01  # roll
        imu_msg.orientation_covariance[4] = 0.01  # pitch
        imu_msg.orientation_covariance[8] = 0.01  # yaw

        self.pub_imu.publish(imu_msg)

        # GPS mesajı
        gps_msg = NavSatFix()
        gps_msg.header.stamp  = now
        gps_msg.latitude      = float(lat)
        gps_msg.longitude     = float(lon)
        gps_msg.status.status = 0 if fix > 0 else -1
        self.pub_gps.publish(gps_msg)

        # Encoder ortalama hız
        vel_msg      = Float32()
        vel_msg.data = (enc_vel_left + enc_vel_right) / 2.0
        self.pub_enc_vel.publish(vel_msg)


def main():
    rclpy.init()
    node = STM32BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
