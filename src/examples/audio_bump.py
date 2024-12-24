'''
audio_bump.py
'''

import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient

from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import AudioNote
from irobot_create_msgs.msg import AudioNoteVector
from irobot_create_msgs.action import AudioNoteSequence
from builtin_interfaces.msg import Duration

namespace = '/robocop'

class AudioNotes():

    def __init__(self):
        self.audionote1 = AudioNote(
            frequency=440,
            max_runtime=Duration(sec=0, nanosec=int(5e8)))
        self.audionote2 = AudioNote(
            frequency=660,
            max_runtime=Duration(sec=0, nanosec=int(5e8)))
        self.audionote3 = AudioNote(
            frequency=880,
            max_runtime=Duration(sec=0, nanosec=int(5e8)))
        self.audionote4 = AudioNote(
            frequency=1000,
            max_runtime=Duration(sec=0, nanosec=int(5e8)))
        self.audionote5 = AudioNote(
            frequency=1100,
            max_runtime=Duration(sec=0, nanosec=int(5e8)))
        
class Bump_Sounds(Node):

    def __init__(self):
        super().__init__('bump_sound')
        print('Hazard detection (subscriber), audio note (publisher), and audio note sequence (action) are all initialized.')
        self.subscription = self.create_subscription(
            HazardDetectionVector,
            f'{namespace}/harzard_detection',
            self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(
            AudioNoteVector,
            f'{namespace}/cmd_autio',
            10)
        self._action_client = ActionClient(
            self,
            AudioNoteSequence,
            f'{namespace}/audio_note_sequence')
        
        self.audio_note_vector = AudioNoteVector()
        self.an = AudioNotes()

    def listener_callback(self, msg):

        def _reaction_to_hit(bumper_location, audio_note):
            self.audio_note_vector.notes = [audio_note]
            print(f'Publishing specific audio note ({bumper_location} bumper hit) to audio note vector.')
            self.publisher.publish(self.audio_note_vector)
            self._action_client.note_sequence = self.audio_note_vector
            print('Sending goal to play audio note vector.')
            self.send_goal()
        
        for detection in msg.detections:
            det = detection.header.frame_id
            if det != "base_link":
                print(det)
                if det == "bump_right":
                    _reaction_to_hit('right', self.an.audionote1)
                elif det == "bump_left":
                    _reaction_to_hit('left', self.an_audionote2)
                elif det == "bump_front_left":
                    _reaction_to_hit('front left', self.an.audionote3)
                elif det == "bump_front_right":
                    _reaction_to_hit('front right', self.an.audionote4)
                elif det == "bump_center":
                    _reaction_to_hit('center', self.an.audionote5)
    
    def send_goal(self):
        goal_msg = AudioNoteSequence.Goal()
        print('waiting for action server to be available...')
        self._action_client.wait_for_server()

        print('Action server available. Sending audio note goal to server.')
        return self._action_client.send_goal_async(goal_msg)
    
def main(args=None):
    rclpy.init(args=args)
    bump_sound = Bump_Sounds()
    try:
        rclpy.spin(bump_sound)
    except KeyboardInterrupt:
        print('\nCaught keyboard interrupt.')
    finally:
        print('Done')
        bump_sound.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
