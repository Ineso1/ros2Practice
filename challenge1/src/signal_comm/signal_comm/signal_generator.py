import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator_node')
        self.signalTopic = self.create_publisher(Float32, 'signal', 10)
        self.timeTopic = self.create_publisher(Int32, 'time', 10)
        self.topic_frequency = 10
        self.topic_period = 1/self.topic_frequency
        self.timer = self.create_timer(self.topic_period, self.timer_callback)

        self.signal_frequency = Float32()
        self.signal = Float32()
        self.time = Int32()
        self.time = 0

        self.get_logger().info('Signal node initialized')


    def timer_callback(self):
        self.signal.data  = math.sin(self.time)
        self.time += self.topic_period
        self.signalTopic.publish(self.signal)
        print('Time: {},\t Sine(t): {}'.format(self.time, self.signal.data))



def main(args = None):
    rclpy.init(args = args)
    signal_generator = SignalGenerator()
    rclpy.spin(signal_generator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()