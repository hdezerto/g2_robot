import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from robp_interfaces.msg import DutyCycles

import math
import numpy as np


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller")
        self.path_subscriber = self.create_subscription(
            Path, "/planned_path", self.path_callback, 10
        )
        self.stop_subscriber = self.create_subscription(
            Bool, "/stop_motion", self.stop_callback, 10
        )
        self.motion_publisher = self.create_publisher(
            DutyCycles, "/motor/duty_cycles", 10
        )
        self.reached_destination_publisher = self.create_publisher(
            Bool, "/reached_destination", 10
        )
        self.reached_waypoint_publisher = self.create_publisher(
            Bool, "/reached_waypoint", 10
        )
        self.current_path = None
        self.current_waypoint_index = 0
        self.obstacle_detected = False

        self.next_goal = PoseStamped()
        self.reached_waypoint = False

        # Control Parameter
        # initialize variables
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0
        self.reset_rotation = True
        self.phase_one = 1
        self.x_0 = 0.0
        self.y_0 = 0.0

        # Parameters (changing allowed)
        self.goal_margin_translational = 0.05
        self.goal_margin_rotational = math.pi / 15

        self.p_rotation = (
            10  # 0 !< p_rotation !< 2base/(h*radius) =  12.599/h h:=sampling time
        )
        self.p_translation = (
            10  # 0 !< p_translation !< 2/(h*radius) = 40.642/h h:=sampling time
        )
        self.p_rotation_two = (
            10  # 0 !< p_rotation_two !< 2*base/(p*h*radius) = 41.999 h:= sampling time
        )
        self.v_damper = 1
        self.w_damper = 1
        self.p = 0.3  # !>0 orientiert sich an einen punkt p meter vor sich

        self.cycle_damping = 0.1

    def path_callback(self, msg: Path):
        self.get_logger().info("Received new path")
        self.current_path = msg
        self.current_waypoint_index = 0
        self.obstacle_detected = False
        self.follow_path()

    def stop_callback(self, msg):
        if msg.data:
            self.get_logger().info("Stop command received")
            self.obstacle_detected = True
            self.stop_robot()

    def follow_path(self):
        if self.current_path is None:
            return
        elif self.position_reached:
            self.get_logger().info("Reached destination")
            return

        while (
            self.current_waypoint_index < len(self.current_path.poses)
            and not self.obstacle_detected
        ):
            waypoint = self.current_path.poses[self.current_waypoint_index]
            self.move_to_waypoint(waypoint)
            if self.reached_waypoint:
                self.current_waypoint_index += 1
                self.reached_waypoint = False
                self.reached_waypoint_publisher.publish(Bool(data=True))
            else:
                self.get_logger().info(
                    "Failed to reach waypoint, stopping path execution"
                )
                break

        if not self.obstacle_detected:
            self.notify_reached_destination(True)
        else:
            self.notify_reached_destination(False)

    def move_to_waypoint(self, waypoint, tolerance=0.1):
        # init
        v = 0
        w = 0

        # Init goal positions if there is a new waypoint
        if self.next_goal != waypoint:
            self.next_goal = waypoint
            self.control_phase = 1
            self.reset_rotation = True
            self.x_g = waypoint.pose.position.x
            self.y_g = waypoint.pose.position.y
            q = waypoint.pose.orientation
            _, _, self.theta_final = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Get the robot's current pose
        current_pose = self.get_current_pos()
        if current_pose is None:
            self.get_logger().error("Failed to get current pose")
            return

        # Extract position and orientation
        current_position = (current_pose[0], current_pose[1])  # (x, y)
        theta = current_pose[2]  # theta

        # Compute the distance and angle to the waypoint
        dx = self.x_g - current_position[0]
        dy = self.y_g - current_position[1]

        # set the goal angle and starting point
        if self.reset_rotation:
            self.theta_g = math.atan2(dy, dx)
            self.x_0 = current_position[0]
            self.y_0 = current_position[1]
            self.reset_rotation = False

        distance = math.sqrt(dx**2 + dy**2)
        delta_theta = self.theta_g - theta  # angle for the robot to face the waypoint

        # Deal with the case where turning the other direction would be faster
        if delta_theta > math.pi:
            delta_theta = delta_theta - 2 * math.pi
        elif delta_theta < -math.pi:
            delta_theta = delta_theta + 2 * math.pi

        if self.control_phase == 1:
            if abs(delta_theta) < self.goal_margin_rotational:
                self.control_phase = 2
                self.get_logger().info("Facing towards the waypoint")
            else:
                w = self.p_rotation * delta_theta
                v = 0
        # Moving towards the waypoint
        elif self.control_phase == 2:
            # Check if the robot has reached the waypoint
            if distance < self.goal_margin_translational:
                # If the robot has reached the final waypoint
                if self.current_waypoint_index == len(self.current_path.poses) - 1:
                    self.get_logger().info("Reached final waypoint")
                    self.control_phase = 3
                    self.get_logger().info(
                        "Reached final waypoint. Now correcting orientation"
                    )
                else:
                    self.get_logger().info("Reached waypoint")
                    self.reached_waypoint = True
            else:
                # Set velocity
                v = self.p_translation * (
                    dx * math.cos(self.theta_g) + dy * math.sin(self.theta_g)
                )
                dp = math.sin(self.theta_g) * (
                    current_position[0] + self.p * math.cos(theta) - self.x_0
                ) - math.cos(self.theta_g) * (
                    current_position[1] + self.p * math.sin(theta) - self.y_0
                )
                w = self.p_rotation_two * dp
        # Correcting orientation at the final waypoint
        elif self.control_phase == 3:
            # Get the angle to the final waypoints orientation
            final_dtheta = self.theta_final - theta
            if final_dtheta > math.pi:
                final_dtheta = final_dtheta - 2 * math.pi
            elif final_dtheta < -math.pi:
                final_dtheta = final_dtheta + 2 * math.pi

            if abs(final_dtheta) < self.goal_margin_rotational:
                self.get_logger().info("Final Waypoint Reached")
                self.reached_waypoint = True
            else:
                w = self.p_rotation * final_dtheta
                v = 0

        self.get_logger().info(f"v: {v}, w: {w}")

        # Damping
        v = v * self.v_damper
        w = w * self.w_damper

        if abs(v) > 1.5:
            v = np.sign(v) * 1.5
        if abs(w) > 1.5:
            w = np.sign(w) * 1.5

        # Convert to duty cycles
        motor_msg = DutyCycles()
        motor_msg.duty_cycle_left = (v - 0.5 * w) * self.cycle_damping
        motor_msg.duty_cycle_right = (v + 0.5 * w) * self.cycle_damping
        motor_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the duty cycles
        self.motion_publisher.publish(motor_msg)
        return

    ###### Integration In Progress ######

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Robot stopped")

    def notify_reached_destination(self, success):
        msg = Bool()
        msg.data = success
        self.reached_destination_publisher.publish(msg)

    def compute_angular_velocity(
        self,
        angular_error,
        previous_angular_error=0.0,
        integral_angular_error=0.0,
        k_p=1.0,
        k_i=0.1,
        k_d=0.1,
    ):
        # PID control for angular velocity
        derivative = angular_error - previous_angular_error
        integral_angular_error += angular_error

        angular_velocity = (
            (k_p * angular_error) + (k_i * integral_angular_error) + (k_d * derivative)
        )

        # Save the current error for the next iteration
        self.previous_angular_error = angular_error

        return angular_velocity

    def compute_linear_velocity(
        self,
        distance,
        previous_distance=0.0,
        integral_distance=0.0,
        k_p=0.5,
        k_i=0.1,
        k_d=0.1,
    ):
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

    def get_current_pos(self):
        # Get transform from 'odom' to 'base_link'
        time = self.get_clock().now().to_msg()
        robot_frame = "map"
        goal_frame = "base_link"

        # Wait for the transform asynchronously
        compared_transform = self.buffer.wait_for_transform_async(
            target_frame=robot_frame, source_frame=goal_frame, time=time
        )
        rclpy.spin_until_future_complete(self, compared_transform, timeout_sec=0.5)

        try:
            # Extract translation (x, y) and rotation (theta)
            transform = compared_transform.result()
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            q = transform.transform.rotation
            _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

            return (x, y, theta)
        except Exception as e:
            self.get_logger().error(f"Failed to get current pose: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)

    try:
        motion_controller = MotionController()
        motion_controller.get_logger().info("MotionController node has started.")

        # Use a MultiThreadedExecutor to enable the use of multiple callbacks in parallel
        executor = MultiThreadedExecutor()
        executor.add_node(motion_controller)
        executor.spin()
    except KeyboardInterrupt:
        motion_controller.get_logger().info("MotionController node is shutting down.")
    finally:
        motion_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
