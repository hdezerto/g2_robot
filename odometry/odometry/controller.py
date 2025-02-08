import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry  # Assuming odometry has a timestamp
from robp_interfaces.msg import DutyCycles
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher for motor duty cycles
        self._pub = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)

        # Initialize the TF broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the /cmd_vel topic
        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        # Subscribe to the /odom topic to get the timestamp
        self.create_subscription(Joy, '/joy', self.odom_callback, 10)

        # Store the latest timestamp
        self.latest_timestamp = None

    def odom_callback(self, msg: Joy):
        """Stores the latest timestamp from the /odom topic."""
        self.latest_timestamp = msg.header.stamp

    def twist_callback(self, msg: Twist):
        """Processes the incoming Twist message and sets duty cycles."""

        v = msg.linear.x / 1.5
        w = msg.angular.z

        # Create a new DutyCycles message
        motor_msg = DutyCycles()
        
        # Define duty cycle values
        motor_msg.duty_cycle_left = 0.05 * (v - w)
        motor_msg.duty_cycle_right = 0.05 * (v + w)

        # Use the latest stored timestamp if available
        if self.latest_timestamp:
            motor_msg.header.stamp = self.latest_timestamp
        else:
            self.get_logger().warn("No timestamp available, using node time.")
            motor_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the message
        self._pub.publish(motor_msg)


def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()