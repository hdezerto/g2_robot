import rclpy
from rclpy.node import Node
from robp_interfaces.msg import DutyCycles

class TestMaxDutyCycle(Node):
    def __init__(self):
        super().__init__('test_max_duty_cycle_node')
        self.publisher = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)
        self.timer = self.create_timer(0.1, self.publish_max_duty_cycle)  # 10Hz frequency

    def publish_max_duty_cycle(self):
        """Publishes the maximum duty cycle to the motors."""
        duty_cycle_msg = DutyCycles()
        # duty_cycle_msg.duty_cycle_left = 1.0  # Maximum duty cycle
        # duty_cycle_msg.duty_cycle_right = 1.0  # Maximum duty cycle
        duty_cycle_msg.duty_cycle_left = 0.5  # Maximum duty cycle
        duty_cycle_msg.duty_cycle_right = 0.5  # Maximum duty cycle
        duty_cycle_msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(duty_cycle_msg)
        self.get_logger().info("Published maximum duty cycle: 1.0 (left), 1.0 (right)")

def main():
    rclpy.init()
    node = TestMaxDutyCycle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
