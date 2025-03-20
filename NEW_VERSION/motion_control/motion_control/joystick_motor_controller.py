import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robp_interfaces.msg import DutyCycles

# ------------- TUNABLE PARAMETERS -------------

# Define max velocity values (from https://index.ros.org/p/teleop_twist_joy/)
MAX_LINEAR_VEL = 0.5  # m/s (adjust for turbo mode if needed)
MAX_ANGULAR_VEL = 0.5  # rad/s (adjust for turbo mode if needed)

DAMPING_FACTOR = 0.1 # Damping factor for the velocities

# ----------------------------------------------


class JoystickMotorController(Node):
    def __init__(self):
        super().__init__('JoystickMotorController_node')

        # Initialize the publisher for motor duty cycles
        self._pub = self.create_publisher(DutyCycles, '/motor/duty_cycles', 1)

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
        #self.get_logger().info(f"Published: {motor_msg.duty_cycle_left}, {motor_msg.duty_cycle_right}")


def main():
    rclpy.init()
    node = JoystickMotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()




# VVV ----------- MAXIMOS CODE BELOW ----------- VVV


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Joy
# from nav_msgs.msg import Odometry  # Assuming odometry has a timestamp
# from robp_interfaces.msg import DutyCycles
# from tf2_ros import TransformBroadcaster
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener


# class Detection(Node):

#     def __init__(self):
#         super().__init__('detection')

#         # Initialize the publisher for motor duty cycles
#         self._pub = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)

#         # Initialize the TF broadcaster
#         self._tf_broadcaster = TransformBroadcaster(self)

#         # Subscribe to the /cmd_vel topic
#         self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

#         # Subscribe to the /odom topic to get the timestamp
#         self.create_subscription(Joy, '/joy', self.odom_callback, 10)

#         # Store the latest timestamp
#         self.latest_timestamp = None

#     def odom_callback(self, msg: Joy):
#         """Stores the latest timestamp from the /odom topic."""
#         self.latest_timestamp = msg.header.stamp

#     def twist_callback(self, msg: Twist):
#         """Processes the incoming Twist message and sets duty cycles."""

#         damping_factor = 0.1

#         v = msg.linear.x   /1.5 #*damping_factor
#         w = msg.angular.z #*damping_factor * 7

        

#         # Create a new DutyCycles message
#         motor_msg = DutyCycles()
        
#         # Define duty cycle values
#         motor_msg.duty_cycle_left = (v - w)*damping_factor #*0.05
#         motor_msg.duty_cycle_right = (v + w)*damping_factor #*0.05
        
#         # wheel_radius = 0.04921
#         # base = 0.3
#         # motor_msg.duty_cycle_left = ((2*v) - base*w)/(2*wheel_radius)
#         # motor_msg.duty_cycle_right = ((2*v) + base*w)/(2*wheel_radius)

#         # Use the latest stored timestamp if available
#         if self.latest_timestamp:
#             motor_msg.header.stamp = self.latest_timestamp
#         else:
#             self.get_logger().warn("No timestamp available, using node time.")
#             motor_msg.header.stamp = self.get_clock().now().to_msg()

#         # Publish the message
#         self._pub.publish(motor_msg)
#         self.get_logger().info(f"Published: {motor_msg.duty_cycle_left}, {motor_msg.duty_cycle_right}")
#         print(self._pub.topic)



# def main():
#     rclpy.init()
#     node = Detection()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()