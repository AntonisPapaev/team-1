#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped, Float32, Bool


class TofNode(Node):
    def __init__(self):
        super().__init__('tof')
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, 10)
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 10)

        self.lane_error = 0.0
        self.lane_valid = False

        self.create_subscription(Float32, f'/{self.vehicle_name}/lane_error', self.lane_error_cb, 10)
        self.create_subscription(Bool, f'/{self.vehicle_name}/lane_valid', self.lane_valid_cb, 10)

        self.declare_parameter('base_speed', 0.20)
        self.declare_parameter('kp', 0.45)
        self.declare_parameter('kd', 0.08)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('max_turn', 0.18)
        self.declare_parameter('tof_stop_distance', 0.20)

        self.base_speed = float(self.get_parameter('base_speed').value)
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.ki = float(self.get_parameter('ki').value)
        self.max_turn = float(self.get_parameter('max_turn').value)
        self.tof_stop_distance = float(self.get_parameter('tof_stop_distance').value)

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

    def check_range(self, msg):
        distance = msg.range

        if distance < self.tof_stop_distance:
            self.stop_moving()
            self.prev_error = 0.0
            self.integral = 0.0
            self.prev_time = self.get_clock().now()
            return

        # Checks if lane data updated recently
        age = (self.get_clock().now() - self.last_lane_time).nanoseconds / 1e9
        if age > 0.5:
            self.lane_valid = False

        if not self.lane_valid:
            if self.lost_lane_behavior == 'search':
                base = 0.08
                turn = self.search_turn
                left = base - turn
                right = base + turn
                left = self.clamp(left, -self.wheel_limit, self.wheel_limit)
                right = self.clamp(right, -self.wheel_limit, self.wheel_limit)
                self.publish_wheels(left, right)
            else:
                self.stop_moving()
            self.integral = 0.0
            self.prev_time = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-3

        e = float(self.lane_error)
        de = (e - self.prev_error) / dt
        self.integral += e * dt

        turn = self.kp * e + self.kd * de + self.ki * self.integral
        turn = self.clamp(turn, -self.max_turn, self.max_turn)

        left = self.base_speed - turn
        right = self.base_speed + turn

        left = self.clamp(left, -self.wheel_limit, self.wheel_limit)
        right = self.clamp(right, -self.wheel_limit, self.wheel_limit)

        self.publish_wheels(left, right)

        self.prev_error = e
        self.prev_time = now

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))
    
    
    def publish_wheels(self, left: float, right: float):
        msg = WheelsCmdStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vel_left = float(left)
        msg.vel_right = float(right)
        self.publisher_.publish(msg)

    def lane_error_cb(self, msg: Float32):
        self.lane_error = float(msg.data)

    def lane_valid_cb(self, msg: Bool):
        self.last_lane_time = self.get_clock().now()
        self.lane_valid = bool(msg.data)



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