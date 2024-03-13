import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from scipy import signal
import numpy as np

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator_node')

        self.declare_parameter('signal_type', 'sine')  
        self.declare_parameter('amplitude', 1.0)
        self.declare_parameter('frequency', 1.0) 
        self.declare_parameter('offset', 0.0)

        self.signal_type = self.get_parameter('signal_type').get_parameter_value().string_value
        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.offset = self.get_parameter('offset').get_parameter_value().double_value

        self.signalTopic = self.create_publisher(Float32, 'signal', 10)
        self.time = 0
        self.topic_period = 0.1 
        self.timer = self.create_timer(self.topic_period, self.timer_callback)

        self.get_logger().info(f'Signal node initialized with {self.signal_type} signal')

    def timer_callback(self):
        time_step = self.frequency * self.time
        if self.signal_type == 'sine':
            # Use numpy for sine wave (since scipy doesn't provide a separate function for sine waves)
            self.signal = self.amplitude * np.sin(2 * np.pi * time_step) + self.offset
        elif self.signal_type == 'square':
            # Use scipy for square wave
            self.signal = self.amplitude * signal.square(2 * np.pi * time_step) + self.offset
        elif self.signal_type == 'sawtooth':
            # Use scipy for sawtooth wave
            self.signal = self.amplitude * signal.sawtooth(2 * np.pi * time_step) + self.offset
        else:
            self.signal = 0 
            self.get_logger().warn(f'Unknown signal type: {self.signal_type}')

        signal_msg = Float32()
        signal_msg.data = self.signal
        self.signalTopic.publish(signal_msg)
        self.get_logger().info(f'Time: {self.time}, Signal: {self.signal}')
        self.time += self.topic_period

def main(args=None):
    rclpy.init(args=args)
    signal_generator = SignalGenerator()
    rclpy.spin(signal_generator)
    signal_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
