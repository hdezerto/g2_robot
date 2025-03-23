import rclpy
from rclpy.node import Node
from detection_interfaces.msg import DetectionMsg

class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')
        self.subscription = self.create_subscription(
            DetectionMsg,
            'detection_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info('Detection Subscriber Node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = DetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()