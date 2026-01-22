#!/usr/bin/python3

import os

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from opencv.opencv_functions import Image, find_latest_image
from opencv.color_hsv import hsv_ranges
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
import numpy as np


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        self.get_logger().info("in init")
        self.output_dir = "/workspace/src/blank_package/include/opencv/images/"
        # self.output_dir = os.path.join(os.path.dirname(__file__), "opencv", "images")
        os.makedirs(self.output_dir, exist_ok=True)
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.counter = 0
        self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.save_image, 10)
        self.publisher = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)

    def save_image(self, msg):
        # self.get_logger().info("in save_image")
        if self.counter % 30 != 0:
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
        error = image.find_error_from_middle()

        # LEDPattern is a custom Duckietown Message
        msg = LEDPattern()
    
        if abs(error) < 10:
            self.get_logger().info("LED off")
            msg.rgb_vals = [ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0), # front left
                        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0), # back right
                        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0), # front right
                        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0), # emt
                        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)] # back left
        elif abs(error) > 0:
            self.get_logger().info("right LED on")
            msg.rgb_vals = [ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0), # front left
                        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0), # back right
                        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0), # front right
                        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0), # emt
                        ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)] # back left
        else:
            self.get_logger().info("left LED on")
            msg.rgb_vals = [ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0), # front left
                        ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # back right
                        ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # front right
                        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0), # emt
                        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)] # back left
        self.publisher.publish(msg)
        self.counter += 1


def main():
    print("in main")
    rclpy.init()
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
