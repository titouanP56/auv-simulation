import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import WorldStatistics
from std_msgs.msg import Float64

class RTFMonitor(Node):
    def __init__(self):
        super().__init__('rtf_monitor')
        self.subscription = self.create_subscription(
            WorldStatistics,
            '/world/stats',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float64, '/world/rtf_average', 10)
        self.rtf_history = []
        self.history_size = 50 # Compute average over last 50 samples
        self.msg_count = 0

    def listener_callback(self, msg):
        self.rtf_history.append(msg.real_time_factor)
        if len(self.rtf_history) > self.history_size:
            self.rtf_history.pop(0)
        
        avg_rtf = sum(self.rtf_history) / len(self.rtf_history)
        
        out_msg = Float64()
        out_msg.data = avg_rtf
        self.publisher_.publish(out_msg)

        self.msg_count += 1
        # Log periodically to avoid spamming the console
        if self.msg_count % 50 == 0:
            self.get_logger().info(f'Average Real Time Factor (RTF): {avg_rtf:.3f}')

def main(args=None):
    rclpy.init(args=args)
    rtf_monitor = RTFMonitor()
    rclpy.spin(rtf_monitor)
    rtf_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
