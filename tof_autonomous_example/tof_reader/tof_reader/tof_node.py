#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, Float32, Bool
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped


class TofNode(Node):
    def __init__(self):
        super().__init__('tof')

        self.vehicle_name = os.getenv('VEHICLE_NAME', 'duckiebot')
        self.get_logger().info(f"VEHICLE_NAME={self.vehicle_name}")

        # Publishers / Subscribers
        self.tof_sub = self.create_subscription(
            Range, f'/{self.vehicle_name}/range', self.check_range, 10
        )
        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10
        )

        self.lane_error = 0.0
        self.lane_valid = False
        self.last_lane_time = self.get_clock().now()

        self.create_subscription(Float32, f'/{self.vehicle_name}/lane_error', self.lane_error_cb, 10)
        self.create_subscription(Bool, f'/{self.vehicle_name}/lane_valid', self.lane_valid_cb, 10)

        # Parameters (give non-zero safe defaults)
        self.lost_lane_behavior = self.declare_parameter('lost_lane_behavior', 'search').value
        self.wheel_limit = float(self.declare_parameter('wheel_limit', 0.6).value)
        self.search_turn = float(self.declare_parameter('search_turn', 0.25).value)

        self.base_speed = float(self.declare_parameter('base_speed', 0.20).value)
        self.kp = float(self.declare_parameter('kp', 0.45).value)
        self.kd = float(self.declare_parameter('kd', 0.08).value)
        self.ki = float(self.declare_parameter('ki', 0.0).value)
        self.max_turn = float(self.declare_parameter('max_turn', 0.18).value)
        self.tof_stop_distance = float(self.declare_parameter('tof_stop_distance', 0.20).value)

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

        self.get_logger().info(f"Subscribing to: /{self.vehicle_name}/range, /{self.vehicle_name}/lane_error, /{self.vehicle_name}/lane_valid")

        self.get_logger().info("INNIT DONE")

    def check_range(self, msg: Range):
        self.get_logger().info("in check range")
        distance = float(msg.range)

        if distance < self.tof_stop_distance:
            self.get_logger().info("in if dist<self.tof")
            self.stop()
            self.prev_error = 0.0
            self.integral = 0.0
            self.prev_time = self.get_clock().now()
            return

        if not self.lane_valid:
            if self.lost_lane_behavior == 'search':
                base = 0.08
                turn = float(self.search_turn)

                left = self.clamp(base - turn, -self.wheel_limit, self.wheel_limit)
                right = self.clamp(base + turn, -self.wheel_limit, self.wheel_limit)
                self.publish_wheels(left, right)
            else:
                self.stop()

            self.integral = 0.0
            self.prev_time = self.get_clock().now()
            return

        # PID lane following
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-3

        e = float(self.lane_error)
        de = (e - self.prev_error) / dt
        self.integral += e * dt

        turn = self.kp * e + self.kd * de + self.ki * self.integral
        turn = self.clamp(turn, -self.max_turn, self.max_turn)

        left = self.clamp(self.base_speed - turn, -self.wheel_limit, self.wheel_limit)
        right = self.clamp(self.base_speed + turn, -self.wheel_limit, self.wheel_limit)

        self.publish_wheels(left, right)

        self.prev_error = e
        self.prev_time = now

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def publish_wheels(self, left: float, right: float):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'tof_node'
        msg.vel_left = float(left)
        msg.vel_right = float(right)
        self.wheels_pub.publish(msg)

    def lane_error_cb(self, msg: Float32):
        self.lane_error = float(msg.data)

    def lane_valid_cb(self, msg: Bool):
        self.get_logger().info(f"lane_valid cb: {msg.data}")
        self.last_lane_time = self.get_clock().now()
        self.lane_valid = bool(msg.data)

    def stop(self):
        self.run_wheels('stop', 0.0, 0.0)

    def run_wheels(self, frame_id: str, vel_left: float, vel_right: float):
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = float(vel_left)
        wheel_msg.vel_right = float(vel_right)
        self.wheels_pub.publish(wheel_msg)


def main():
    rclpy.init()
    node = TofNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
