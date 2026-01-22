#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped
from opencv.constants import DEVIATION as error


class TofNode(Node):
    def __init__(self):
        super().__init__('tof')
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, 10)
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)

    def check_range(self, msg):
        distance = msg.range
        if distance >= 0.2:
            self.move_forward()
        else:
            self.stop()

    def move_forward(self):
        target_left, target_right = 0.2, 0.192
        if abs(error) < 15:
            self.get_logger().info("straight")
            target_left, target_right = 0.2, 0.192
        elif error < 0:
            self.get_logger().info("right")
            target_left, target_right = 0.0, 0.15
        else:
            self.get_logger().info("left")
            target_left, target_right = 0.15, 0.0

        self.run_wheels('forward_callback', target_left, target_right)

    def stop(self):
        self.run_wheels('stop_callback', 0.0, 0.0)

    def run_wheels(self, frame_id, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheels_pub.publish(wheel_msg)


def main():
    rclpy.init()
    tof = TofNode()
    rclpy.spin(tof)
    rclpy.shutdown()


if __name__ == '__main__':
    main()