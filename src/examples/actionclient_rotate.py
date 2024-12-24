'''
actionclient_rotate.py
'''

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from irobot_create_msgs.action import RotateAngle

namespace = '/robocop'

class RotateActionClient(Node):

    def __init__(self):
        super().__init__('rotate_action_client')
        self._action_client = ActionClient(
            self, 
            RotateAngle,
            f'{namespace}/rotate_angle')

    def send_goal(self, angle, max_rotation_speed):
        goal_msg = RotateAngle.Goal()
        goal_msg.angle = angle
        goal_msg.max_rotation_speed = max_rotation_speed

        print('waiting for action server to be avaiblable...')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        print('Checking if goal was accepted or rejected...')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        print('Shutting down rotate action client node.')
        rclpy.shutdown()

def main(args=None):
    angle = 6.28
    speed = 0.5
    rclpy.init(args=args)
    action_client = RotateActionClient()
    try:
        action_client.send_goal(angle, speed)
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info(
            '\nKeyboard interrupt detected. Shutting down...')
    finally:
        action_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()