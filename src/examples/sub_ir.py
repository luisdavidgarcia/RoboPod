'''
sub_ir.py
Subcriber Example
'''

import rclpy
import sys

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import IrIntensityVector

namespace = '/robocop'

class IRSubscriber(Node):
    def __init__(self):
        super().__init__('IR_subscriber')
        print('Creating subscription to the IrIntensityVector type over the /ir_intensity topic')
        self.subscription = self.create_subscription(
                IrIntensityVector, 
                namespace + '/ir_intensity', 
                self.listener_callback, 
                qos_profile_sensor_data)

    def printIR(self, msg):
        '''
        :type msg: IrIntesity
        :rtype: None
        '''
        print('Printing IR sensor readings: ')
        for reading in msg.readings:
            val = reading.value
            print('IR sensor: ' + str(val))

    def listener_callback(self, msg:IrIntensityVector):
        print('Now listening to IR sensor readings it hears...')
        self.printIR(msg)

def main(args=None):
    rclpy.init(args=args)
    IR_subscriber = IRSubscriber()
    print('Callbacks are called.')
    try:
        rclpy.spin(IR_subscriber)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt')
    finally:
        print('Done')
        IR_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

