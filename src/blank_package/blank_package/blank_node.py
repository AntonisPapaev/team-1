#!/usr/bin/python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from opencv.opencv_functions import Image, find_latest_image
from opencv.color_hsv import hsv_ranges

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.get_logger().info("in init")
        self.output_dir = "/workspace/src/blank_package/include/opencv/images/"
        os.makedirs(self.output_dir, exist_ok=True)
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.counter = 0
        self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.save_image, 10)

    def save_image(self, msg):
        self.get_logger().info("in save_image")
        if self.counter % 30 != 0:
            self.counter += 1
            return
        with open(self.output_dir + str(self.counter) + '.jpg', 'wb') as f:
            self.get_logger().info(f'Saving image {self.counter}')
            f.write(msg.data)
        self.counter += 1
        image_path = find_latest_image()
        image = Image(image_path)
        error = image.find_error_from_middle()
        if error < 10:
            self.get_logger().info("LED on")
        else:
            self.get_logger().info("LED off")


def main():
    print("in main")
    rclpy.init()
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
