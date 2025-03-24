import rclpy
from rclpy.node import Node
from detection_interfaces.msg import DetectionMsg

class DetectionPublisher(Node):
    def __init__(self):
        super().__init__('detection_publisher')
        self.publisher_ = self.create_publisher(DetectionMsg, 'detection_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publish every second
        self.get_logger().info('Detection Publisher Node has been started.')

    def publish_message(self):
        msg = DetectionMsg()
        msg.type = "OBJECT"
        msg.cat = 1
        msg.x = 1.0
        msg.y = 2.0
        msg.theta = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = DetectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()