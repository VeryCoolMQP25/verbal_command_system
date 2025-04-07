import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
import os
import json
import time
from pyt2s.services import stream_elements
from pydub import AudioSegment
from io import BytesIO
import simpleaudio as sa

def say(text):
    try:
        data = stream_elements.requestTTS(text, stream_elements.Voice.Joanna.value)
        audio = AudioSegment.from_mp3(BytesIO(data))
        play_obj = sa.play_buffer(
            audio.raw_data, 
            num_channels=audio.channels, 
            bytes_per_sample=audio.sample_width, 
            sample_rate=audio.frame_rate
        )
        # Wait until audio finishes playing
        play_obj.wait_done()
        time.sleep(0.2)
    except Exception as e:
        print(f"TTS Error: {e}")

class GoalProximityNode(Node):
    def __init__(self, audio_stream=None):
        super().__init__('goal_proximity_node')
        self.subscription = self.create_subscription(
            Int32,
            '/check_goal_proximity',
            self.proximity_callback,
            10
        )
        self.arrived = False
        self.audio_stream = audio_stream
        self.get_logger().info('Goal proximity node initialized')
        
    def proximity_callback(self, msg):
        self.get_logger().info(f'Received proximity message: {msg.data}')
        if msg.data == 1:
            self.arrived = True
            self.get_logger().info('Arrived at destination!')
            if self.audio_stream:
                self.audio_stream.stop_stream()
                say("We have arrived at the destination!")
                self.audio_stream.start_stream()