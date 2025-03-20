import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

import math


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.path_subscriber = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        self.stop_subscriber = self.create_subscription(
            Bool,
            '/stop_motion',
            self.stop_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reached_destination_publisher = self.create_publisher(Bool, '/reached_destination', 10)
        self.current_path = None
        self.current_waypoint_index = 0
        self.obstacle_detected = False


    def path_callback(self, msg):
        self.get_logger().info('Received new path')
        self.current_path = msg
        self.current_waypoint_index = 0
        self.obstacle_detected = False
        self.follow_path()


    def stop_callback(self, msg):
        if msg.data:
            self.get_logger().info('Stop command received')
            self.obstacle_detected = True
            self.stop_robot()


    def follow_path(self):
        if self.current_path is None:
            return

        while self.current_waypoint_index < len(self.current_path.poses) and not self.obstacle_detected:
            waypoint = self.current_path.poses[self.current_waypoint_index]
            if self.move_to_waypoint(waypoint):
                self.current_waypoint_index += 1
            else:
                self.get_logger().info('Failed to reach waypoint, stopping path execution')
                break

        if not self.obstacle_detected:
            self.notify_reached_destination(True)
        else:
            self.notify_reached_destination(False)


    def move_to_waypoint(self, waypoint, tolerance=0.1):
        # Get the robot's current pose
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error('Failed to get current pose')
            return False
    
        # Extract position and orientation
        current_position = (current_pose[0], current_pose[1])  # (x, y)
        current_orientation = current_pose[2]  # theta
    
        # Compute the distance and angle to the waypoint
        dx = waypoint.pose.position.x - current_position[0]
        dy = waypoint.pose.position.y - current_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_waypoint = math.atan2(dy, dx)
    
        # Initialize PID variables for linear velocity
        previous_distance = 0.0
        integral_distance = 0.0
    
        # Initialize PID variables for angular velocity
        self.previous_angular_error = 0.0
        integral_angular_error = 0.0
    
        # Loop until the robot reaches the waypoint or an obstacle is detected
        while distance > tolerance and not self.obstacle_detected:
            # Compute the angular error
            angular_error = angle_to_waypoint - current_orientation
    
            # Create a Twist message for velocity control
            twist = Twist()
            twist.angular.z = self.compute_angular_velocity(
                angular_error,
                self.previous_angular_error,
                integral_angular_error
            )
    
            if abs(angular_error) < 0.1:  # Only move forward if roughly aligned
                twist.linear.x, integral_distance = self.compute_linear_velocity(
                    distance,
                    previous_distance,
                    integral_distance
                )
            else:
                twist.linear.x = 0.0  # Stop forward motion if not aligned
    
            # Publish the velocity command
            self.cmd_vel_publisher.publish(twist)
    
            # Update the distance and angle to the waypoint
            current_pose = self.get_current_pose()
            if current_pose is None:
                self.get_logger().error('Failed to get current pose during motion')
                return False
            current_position = (current_pose[0], current_pose[1])
            current_orientation = current_pose[2]
            dx = waypoint.pose.position.x - current_position[0]
            dy = waypoint.pose.position.y - current_position[1]
            previous_distance = distance  # Update the previous distance
            distance = math.sqrt(dx**2 + dy**2)
            angle_to_waypoint = math.atan2(dy, dx)
    
        # Stop the robot when the waypoint is reached
        self.stop_robot()
        return True


    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Robot stopped')


    def notify_reached_destination(self, success):
        msg = Bool()
        msg.data = success
        self.reached_destination_publisher.publish(msg)


    def compute_angular_velocity(self, angular_error, previous_angular_error=0.0, integral_angular_error=0.0, k_p=1.0, k_i=0.1, k_d=0.1):
        # PID control for angular velocity
        derivative = angular_error - previous_angular_error
        integral_angular_error += angular_error
    
        angular_velocity = (k_p * angular_error) + (k_i * integral_angular_error) + (k_d * derivative)
    
        # Save the current error for the next iteration
        self.previous_angular_error = angular_error
    
        return angular_velocity

    
    def compute_linear_velocity(self, distance, previous_distance=0.0, integral_distance=0.0, k_p=0.5, k_i=0.1, k_d=0.1):
        # Compute the proportional term
        proportional = k_p * distance

        # Compute the integral term
        integral_distance += distance
        integral = k_i * integral_distance

        # Compute the derivative term
        derivative = distance - previous_distance
        derivative_term = k_d * derivative

        # Combine the PID terms
        linear_velocity = proportional + integral + derivative_term

        return linear_velocity, integral_distance


    def get_current_pose(self):
        try:
            # Lookup the transform from 'odom' to 'base_link'
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())

            # Extract translation (x, y) and rotation (yaw)
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            return (x, y, yaw)
        except Exception as e:
            self.get_logger().error(f"Failed to get current pose: {e}")
            return None





def main(args=None):
    rclpy.init(args=args)

    try:
        motion_controller = MotionController()
        motion_controller.get_logger().info('MotionController node has started.')

        # Use a MultiThreadedExecutor to enable the use of multiple callbacks in parallel
        executor = MultiThreadedExecutor()
        executor.add_node(motion_controller)
        executor.spin()
    except KeyboardInterrupt:
        motion_controller.get_logger().info('MotionController node is shutting down.')
    finally:
        motion_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()