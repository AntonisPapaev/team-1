#!/usr/bin/python3

import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from opencv.opencv_functions import Image, find_latest_image
from opencv.color_hsv import hsv_ranges
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA, Float32, Bool
import numpy as np
# from ....pidauto.robot.robot.main import RobotController



class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.get_logger().info("in init")
        self.output_dir = "/workspace/src/blank_package/include/opencv/images/"
        # self.output_dir = os.path.join(os.path.dirname(__file__), "opencv", "images")
        os.makedirs(self.output_dir, exist_ok=True)
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        self.declare_parameter('error_alpha', 0.20)
        self.declare_parameter('max_jump_norm', 0.25)
        self.declare_parameter('valid_on', 3)
        self.declare_parameter('valid_off', 2)  
        self.declare_parameter('min_black_pixels', 1500)

        self.error_alpha = float(self.get_parameter('error_alpha').value)
        self.max_jump_norm = float(self.get_parameter('max_jump_norm').value)
        self.valid_on = int(self.get_parameter('valid_on').value)
        self.valid_off = int(self.get_parameter('valid_off').value)
        self.min_black_pixels = int(self.get_parameter('min_black_pixels').value)

        self.filtered_error = 0.0
        self.valid_count = 0
        self.invalid_count = 0
        self.is_valid = False

        self.counter = 0
        self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.save_image, 10)
        self.publisher = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)

        self.lane_error_pub = self.create_publisher(Float32, f'/{self.vehicle_name}/lane_error', 10)
        self.lane_valid_pub = self.create_publisher(Bool, f'/{self.vehicle_name}/lane_valid', 10)

    def save_image(self, msg):
        # self.get_logger().info("in save_image")
        if self.counter % 1 != 0:
            self.counter += 1
            return
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().error("Failed to decode image")
            self.counter += 1
            return
        cv2.imwrite(os.path.join(self.output_dir, f"{self.counter}.jpg"), img)  # remove later
        # with open(self.output_dir + str(self.counter) + '.jpg', 'wb') as f:
        #     self.get_logger().info(f'Saving image {self.counter}')
        #     f.write(msg.data)
        # image_path = find_latest_image()
        # img = cv2.imread(image_path)
        image = Image(img)
        norm_error, valid_raw, black_pixels = image.find_error_from_middle()

        valid_this_frame = bool(valid_raw) and (black_pixels >= self.min_black_pixels)

        if valid_this_frame:
            self.valid_count += 1
            self.invalid_count = 0
        else:
            self.invalid_count += 1
            self.valid_count = 0

        if (not self.is_valid) and (self.valid_count >= self.valid_on):
            self.is_valid = True
        if self.is_valid and (self.invalid_count >= self.valid_off):
            self.is_valid = False

        self.lane_valid_pub.publish(Bool(data=self.is_valid))

        if not self.is_valid:
            return

        if abs(norm_error - self.filtered_error) > self.max_jump_norm:
            self.lane_valid_pub.publish(Bool(data=False))
            return

        self.filtered_error = (1.0 - self.error_alpha) * self.filtered_error + self.error_alpha * norm_error

        self.lane_error_pub.publish(Float32(data=float(self.filtered_error)))
        led_msg = LEDPattern()

        if abs(self.filtered_error) < 0.10:
            self.get_logger().info("LED ON")
            led_msg.rgb_vals = [ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # front left
                            ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # back right
                            ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # front right
                            ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # emt
                            ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)]  # back left
        elif self.filtered_error > 0:
            self.get_logger().info("left LED on")
            led_msg.rgb_vals = [ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # front left
                            ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # back right
                            ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # front right
                            ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # emt
                            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)]  # back left
        else:
            self.get_logger().info("right LED on")
            led_msg.rgb_vals = [ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # front left
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # back right
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # front right
                            ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # emt
                            ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)]  # back left
        self.publisher.publish(led_msg)
        self.counter += 1


def main():
    print("in main")
    rclpy.init()
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
