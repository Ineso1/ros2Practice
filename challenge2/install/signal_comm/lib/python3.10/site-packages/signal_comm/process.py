import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SignalProcessor(Node):
    def __init__(self):
        super().__init__('signal_processor')
        self.signal_subscription = self.create_subscription(Float32, 'signal', self.signal_listener_callback, 10)
        self.proc_signal_publisher = self.create_publisher(Float32, 'proc_signal', 10)
        self.get_logger().info('Listener node initialized')

    def signal_listener_callback(self, signal):
        # Process the received signal
        offset = 1.0
        amplitude_scale = 1.0
        phase_shift = math.pi

        processed_signal_value = offset + amplitude_scale * signal.data * math.cos(phase_shift)

        # Publish the processed signal
        proc_signal_msg = Float32()
        proc_signal_msg.data = processed_signal_value
        self.proc_signal_publisher.publish(proc_signal_msg)

        # Log data
        self.get_logger().info('Original Signal: {}, Processed Signal: {}'.format(signal.data, processed_signal_value))


def main(args=None):
    rclpy.init(args=args)
    signal_processor = SignalProcessor()
    rclpy.spin(signal_processor)
    signal_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
