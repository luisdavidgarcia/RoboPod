'''
pub_lightring.py
'''

import time
import rclpy
from rclpy.node import Node

from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds

namespace = '/robocop'

class LEDPublisher(Node):

    def __init__(self):
        super().__init__('led_publisher')

        self.lights_publisher = self.create_publisher(
            LightringLeds,
            f'{namespace}/cmd_lightring',
            10)
        
        self.lightring = LightringLeds()
        self.lightring.override_system = True

        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self, r=255, g=255, b=255):
        print(f'Changing lights to r={r}, g={g}, b={b}')
        self.lightring.leds = [
            LedColor(red=r, green=g, blue=b),
            LedColor(red=r, green=g, blue=b),
            LedColor(red=r, green=g, blue=b),
            LedColor(red=r, green=g, blue=b),
            LedColor(red=r, green=g, blue=b),
            LedColor(red=r, green=g, blue=b)]
        self.lights_publisher.publish(self.lightring)

    def reset(self):
        self.lightring.override_system = False
        white = [
            LedColor(red=255, green=255, blue=255),
            LedColor(red=255, green=255, blue=255),
            LedColor(red=255, green=255, blue=255),
            LedColor(red=255, green=255, blue=255),
            LedColor(red=255, green=255, blue=255),
            LedColor(red=255, green=255, blue=255)]
        self.lightring.leds = white
        self.lights_publisher.publish(self.lightring)

def main(args=None):
    rclpy.init(args=args)
    led_publisher = LEDPublisher()
    try:
        rclpy.spin(led_publisher)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print('Done')
        
        if rclpy.ok():
            led_publisher.reset()
        
        led_publisher.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()