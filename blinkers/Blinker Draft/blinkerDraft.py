def publish_pattern(self):
        # LEDPattern is a custom Duckietown Message
        msg = LEDPattern()

        if self.counter % 3 == 0:
            # ColorRGBA is a standard message of ROS2
            pattern = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        elif self.counter % 3 == 1:
            pattern = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        else:
            pattern = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)

        # 5 Leds to fill
        msg.rgb_vals = [pattern] * 5

        self.publisher.publish(msg)
        self.counter += 1