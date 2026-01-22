import numpy as np
import time
import cv2
import os
import re
from opencv.color_hsv import hsv_ranges
from opencv_functions import Image


img = Image()

def publish_pattern(self):
        # LEDPattern is a custom Duckietown Message
        msg = LEDPattern()

        if self.counter % 3 == 0:
            # ColorRGBA is a standard message of ROS2
            msg.rgb_vals = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # front left
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # back right
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # front right
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # emt
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)] # back left
        elif self.counter % 3 == 1:
            msg.rgb_vals = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # front left
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # back right
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # front right
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # emt
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)] # back left
        else:
            msg.rgb_vals = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # front left
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # back right
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # front right
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # emt
                            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)] # back left

        # 5 Leds to fill
        msg.rgb_vals = [ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # front left
                        ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # back right
                        ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # front right
                        ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0), # emt
                        ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)] # back left

        self.publisher.publish(msg)
        self.counter += 1