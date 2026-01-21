#!/usr/bin/env python3
import os
import sys
import tty
import termios
import select
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DispatcherKeyboard(Node):
    def __init__(self):
        super().__init__('dispatcher_keyboard')

        self.vehicle_name = os.getenv("VEHICLE_NAME")
        self.user = os.getenv("USER_NAME")

        self.publisher = self.create_publisher(
            String,
            f'/{self.user}/{self.vehicle_name}/command',
            10
        )

        self.last_motion_cmd = 's'
        self.last_key_time = time.monotonic()
        self.deadman_timeout = 0.20

        self.publish_hz = 30.0
        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)
        

        self.get_logger().info("Keyboard control ready (WASD / arrows, space=STOP)")

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)

    def on_timer(self):
        if (time.monotonic() - self.last_key_time) > self.deadman_timeout:
            self.last_motion_cmd = 's'

        self.send_command(self.last_motion_cmd)


class TTYKeyReader:
    def __init__(self):
        self.fd = os.open("/dev/tty", os.O_RDONLY)
        os.set_blocking(self.fd, False)
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def close(self):
        if self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            os.close(self.fd)
            self.fd = None

    def read_key(self) -> str:
        r, _, _ = select.select([self.fd], [], [], 0.0)
        if not r:
            return None 
        ch1 = os.read(self.fd, 1).decode(errors="ignore")
        if ch1 == "\x1b":  # ESC
            ch2 = os.read(self.fd, 1).decode(errors="ignore")
            ch3 = os.read(self.fd, 1).decode(errors="ignore")
            return ch1 + ch2 + ch3
        return ch1

def main(args=None):
    rclpy.init(args=args)
    node = DispatcherKeyboard()
    reader = TTYKeyReader()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = reader.read_key()

            if key is None:
                continue
            
            node.last_key_time = time.monotonic()

            if key in ('w', 'W'):
                node.last_motion_cmd = 'f'
            elif key in ('s', 'S'):
                node.last_motion_cmd = 'b'
            elif key in ('a', 'A'):
                node.last_motion_cmd = 'l'
            elif key in ('d', 'D'):
                node.last_motion_cmd = 'r'
            elif key == ' ':
                node.last_motion_cmd = 's'
            elif key == 'b':
                node.send_command('bl')
            elif key == 'g':
                node.send_command('gl')
            elif key == 'r':
                node.send_command('rl')
            elif key == 'p':
                node.send_command('sol')
            elif key == '\x03':  # Ctrl+C
                break

    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
