#!/usr/bin/python3
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from duckietown_msgs.msg import WheelsCmdStamped
from rclpy.time import Duration
from ././././src/blank_package/blank_package/blank_node.py import DEVIATION as error


class RobotController(Node):

    def __init__(self):
        self.vehicle_name = os.getenv("VEHICLE_NAME")
        self.user = os.getenv("USER_NAME")

        super().__init__('controller')

        self.target_left = 0.0
        self.target_right = 0.0
        self.curr_left = 0.0
        self.curr_right = 0.0
        self.last_cmd_time = self.get_clock().now()


        self.subscription = self.create_subscription(
            String,
            f'/{self.user}/{self.vehicle_name}/command',
            self.command_callback,
            10
        )
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 1)

        self.deadman_timeout = 0.20 
        self.control_period = 0.03
        self.max_delta_per_tick = 0.08 
        self.timer = self.create_timer(self.control_period, self.control_loop)

        self.get_logger().info("The duckiebot controller initialized and waiting for commands...")

    def command_callback(self, msg):
        command = msg.data.lower().strip()
        self.last_cmd_time = self.get_clock().now()

        if abs(error) < 15:
            self.target_left, self.target_right = 0.5, 0.48
        elif command == 'b':
            self.target_left, self.target_right = -0.5, -0.48
        elif error < 0:
            self.target_left, self.target_right = 0.0, 0.25
        else:
            self.target_left, self.target_right = 0.25, 0.0

    def run_wheels(self, frame_id, vel_left, vel_right):
        
        vel_left = max(-1.0, min(1.0, vel_left))
        vel_right = max(-1.0, min(1.0, vel_right))
        
        wheel_msg = WheelsCmdStamped()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right
        self.wheels_pub.publish(wheel_msg)

    def control_loop(self):
        now = self.get_clock().now()

        if (now - self.last_cmd_time) > Duration(seconds=self.deadman_timeout):
            self.target_left = 0.0
            self.target_right = 0.0

        self.curr_left = self._ramp(self.curr_left, self.target_left, self.max_delta_per_tick)
        self.curr_right = self._ramp(self.curr_right, self.target_right, self.max_delta_per_tick)

        self.run_wheels("control_loop", self.curr_left, self.curr_right)

    @staticmethod
    def _ramp(curr, target, max_delta):
        delta = target - curr
        if delta > max_delta:
            return curr + max_delta
        if delta < -max_delta:
            return curr - max_delta
        return target


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
