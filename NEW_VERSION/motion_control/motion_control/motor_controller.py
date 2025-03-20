import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robp_interfaces.msg import DutyCycles

# ------------- TUNABLE PARAMETERS -------------

MAX_LINEAR_VEL = 1.5  # [m/s] (tested experimentally)
MAX_ANGULAR_VEL = 5.1  # [rad/s] computed as: MAX_LINEAR_VEL / WHEEL_BASE

DAMPING_FACTOR = 0.5 # Damping factor for the velocities

# ----------------------------------------------


class MotorController(Node):
    def __init__(self):
        super().__init__('MotorController_node')

        # Initialize the publisher for motor duty cycles
        self._pub = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)

        # Subscribe to the /cmd_vel topic
        self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
    

    def twist_callback(self, msg: Twist):
        """Processes the incoming Twist message and sets duty cycles."""
        
        # Normalize velocities to [-1, 1]
        v = msg.linear.x / MAX_LINEAR_VEL
        w = msg.angular.z / MAX_ANGULAR_VEL

        # Create a new DutyCycles message
        motor_msg = DutyCycles()
        
        # Define duty cycle values withing the range [-1, 1]
        motor_msg.duty_cycle_left = max(min(v - w, 1.0), -1.0) * DAMPING_FACTOR
        motor_msg.duty_cycle_right = max(min(v + w, 1.0), -1.0) * DAMPING_FACTOR
        
        motor_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the message
        self._pub.publish(motor_msg)
        #self.get_logger().info(f"Published: {motor_msg.duty_cycle_left}, {motor_msg.duty_cycle_right}") # DEBUG


def main():
    rclpy.init()
    node = MotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
