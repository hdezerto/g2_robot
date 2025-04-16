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
    """
    A ROS2 node for controlling the motion of a robot. The MotionController class subscribes to path and stop commands,
    and publishes motor duty cycles to move the robot along a planned path while avoiding obstacles.
    Simple proportional control with 3 phases is used to move the robot to each waypoint in the path.
    Phase 1: Rotate towards the waypoint.
    Phase 2: Move towards the waypoint.
    Phase 3: Correct orientation at the final waypoint.

    ATTENTION:
    - After reaching a waypoint, map should be checked for new obstacles and path should possibly be replanned. (Not implemented in this node)
    - Currently no local obstacle avoidance is implemented.

    TODO:
    - Full Turn after reaching a waypoint?
    - Add lidar check before movement?

    Attributes:
        path_subscriber (Subscription): Subscriber for the planned path.
        stop_subscriber (Subscription): Subscriber for stop motion command.
        motion_publisher (Publisher): Publisher for motor duty cycles.
        reached_destination_publisher (Publisher): Publisher for notifying when the destination is reached.
        reached_waypoint_publisher (Publisher): Publisher for notifying when a waypoint is reached.
        current_path (Path): The current path the robot is following.
        current_waypoint_index (int): Index of the current waypoint in the path.
        obstacle_detected (bool): Flag indicating if an obstacle is detected.
        next_goal (PoseStamped): The next waypoint pose.
        reached_waypoint (bool): Flag indicating if the current waypoint is reached.
        stop_robot (bool): Flag indicating if the robot should stop.
        x_g (float): Goal x-coordinate.
        y_g (float): Goal y-coordinate.
        theta_g (float): Goal orientation.
        reset_rotation (bool): Flag indicating if the rotation should be reset.
        phase_one (int): Control phase.
        x_0 (float): Initial x-coordinate.
        y_0 (float): Initial y-coordinate.
        goal_margin_translational (float): Translational goal margin.
        goal_margin_rotational (float): Rotational goal margin.
        p_rotation (float): Proportional gain for rotation.
        p_translation (float): Proportional gain for translation.
        p_rotation_two (float): Proportional gain for rotation in phase two.
        v_damper (float): Velocity damper.
        w_damper (float): Angular velocity damper.
        p (float): Proportional gain for orientation.
        cycle_damping (float): Damping factor for duty cycles.
    Methods:
        path_callback(msg: Path): Callback function for receiving a new path.
        stop_callback(msg: Bool): Callback function for receiving a stop command.
        follow_path(): Follows the current path by moving to each waypoint.
        move_to_waypoint(waypoint, tolerance=0.1): Moves the robot to the specified waypoint.
        stop_robot(): Stops the robot by setting motor duty cycles to zero.
        notify_reached_destination(success: bool): Notifies if the destination is reached.
        get_current_pos(): Gets the current position and orientation of the robot.
    """

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
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.current_path = None
        self.current_waypoint_index = 0
        self.obstacle_detected = False
        self.timer = self.create_timer(0.07, self.follow_path)

        self.next_goal = PoseStamped()
        self.reached_waypoint = False
        self.stop_robot = False

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
        self.goal_margin_rotational = math.pi / 32

        self.p_rotation_one = (
            10  # 0 !< p_rotation !< 2base/(h*radius) =  12.599/h h:=sampling time
        )
        self.p_translation_one = 5
        self.p_translation_two = (
            25  #  p_translation !< 2/(h*radius) = 40.642/h h:=sampling time
        )
        self.p_rotation_two = (
            10  # 0 !< p_rotation_two !< 2*base/(p*h*radius) = 41.999 h:= sampling time
        )
        self.v_damper = 1
        self.w_damper = 1
        self.p = 0.3  # !>0 orientiert sich an einen punkt p meter vor sich

        self.stuck_check = 0
        self.last_distance = 0
        self.last_delta_theta = 0
        self.stuck_parameter = 0

        self.accuracy_check = 0

        self.cycle_damping = 0.1

    def path_callback(self, msg: Path):
        self.get_logger().info("Received new path")

        self.current_path = msg
        self.current_path.poses.pop(0)
        self.current_waypoint_index = 0
        self.obstacle_detected = False
        self.stop_robot = False
        self.reached_waypoint = False
        self.follow_path()

    def stop_callback(self, msg):
        self.stop_robot = msg.data
        if msg.data:
            self.get_logger().info("Stop command received")
            self.stop()

    def follow_path(self):
        if self.current_path is None:
            return

        if (
            self.current_waypoint_index < len(self.current_path.poses)
            and not self.obstacle_detected
            and not self.stop_robot
        ):
            waypoint = self.current_path.poses[self.current_waypoint_index]
            self.move_to_waypoint(waypoint)
            if self.reached_waypoint:
                self.current_waypoint_index += 1
                self.reached_waypoint = False
                self.reached_waypoint_publisher.publish(Bool(data=True))
        elif self.current_waypoint_index == len(self.current_path.poses):
            if not self.obstacle_detected and not self.stop_robot:
                self.notify_reached_destination(True)
            else:
                self.notify_reached_destination(False)
            self.get_logger().info("Path execution finished")

    def move_to_waypoint(self, waypoint):
        """
        Moves the robot to the specified waypoint.
        This function controls the robot's movement towards a given waypoint by
        adjusting its velocity and angular velocity. This is then transformed into
        duty cycles for the motors.
        The movement is divided into three phases:
        1. Rotating towards the waypoint.
        2. Moving towards the waypoint.
        3. Correcting orientation at the final waypoint.

        If the robot is not at the desired position but the input signal is too small for the robot to move, iteratively de   `crease the damping factor.
        Args:
            waypoint (PoseStamped): The target waypoint to move to.
        Returns:
            None
        """

        # init
        v = 0
        w = 0
        stuck_parameter = 0

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

        # Rotating towards the waypoint
        if self.control_phase == 1:
            if abs(delta_theta) < self.goal_margin_rotational:
                self.control_phase = 2
                self.reset_rotation = True
                self.stuck_check = 0
                self.get_logger().info("Facing towards the waypoint. Now moving.")
            else:
                if abs(self.last_delta_theta - delta_theta) < 0.0005:
                    self.stuck_check += 1
                    self.get_logger().info(f"Not turning.")
                else:
                    self.stuck_check = 0
                if self.stuck_check > 5:
                    stuck_parameter = (self.stuck_check - 5) * 0.01
                    self.get_logger().info(
                        f"Stuck. Increasing damping factor to {stuck_parameter}"
                    )

                # Set velocities
                w = self.p_rotation_one * delta_theta
                v = 0
                # If robot should turn on the spot - parameter might need to be adjusted
                # v = self.p_translation_one * (math.cos(delta_theta) * (self.x_0 - current_position[0]) + math.sin(delta_theta) * (self.y_0 - current_position[1]))
        # Moving towards the waypoint
        elif self.control_phase == 2:
            # Check if the robot has reached the waypoint
            if distance < self.goal_margin_translational:
                self.stuck_check = 0

                # If the robot has reached the final waypoint, move to phase 3
                if self.current_waypoint_index == len(self.current_path.poses) - 1:
                    if self.accuracy_check > 5:
                        self.control_phase = 3
                        self.get_logger().info(
                            "Reached final waypoint. Now correcting orientation"
                        )
                    else:
                        self.accuracy_check += 1
                else:
                    self.get_logger().info("Reached waypoint. Moving to next waypoint.")
                    self.reached_waypoint = True
            else:
                # Check if robot is stuck
                if abs(self.last_distance - distance) < 0.0005:
                    self.stuck_check += 1
                    self.get_logger().info(f"Not moving.")
                else:
                    self.stuck_check = 0
                if self.stuck_check > 5:
                    stuck_parameter = (self.stuck_check - 5) * 0.02
                    self.get_logger().info(
                        f"Stuck. Increasing damping factor to {stuck_parameter}"
                    )
                elif self.stuck_check > 10:
                    self.get_logger().warn(
                        f"Could not reach destination with remaining distance: {distance}\n Moving On."
                    )
                    if self.current_waypoint_index == len(self.current_path.poses) - 1:
                        self.control_phase = 3
                        self.get_logger().info(
                            ">Reached< final waypoint. Now correcting orientation"
                        )
                    else:
                        self.get_logger().info(
                            ">Reached< waypoint. Moving to next waypoint."
                        )
                        self.reached_waypoint = True
                    self.accuracy_check = 0

                # Set velocity
                v = self.p_translation_two * (
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
            self.stuck_check = 0
            # Get the angle to the final waypoints orientation
            final_dtheta = self.theta_final - theta
            if final_dtheta > math.pi:
                final_dtheta = final_dtheta - 2 * math.pi
            elif final_dtheta < -math.pi:
                final_dtheta = final_dtheta + 2 * math.pi

            if abs(final_dtheta) < self.goal_margin_rotational:
                self.get_logger().info("Final Waypoint Reached")
                self.stuck_check = 0
                self.reached_waypoint = True
            else:
                if self.last_delta_theta == delta_theta:
                    self.stuck_check += 1
                    self.get_logger().info(f"Not turning.")
                else:
                    self.stuck_check = 0
                if self.stuck_check > 5:
                    stuck_parameter = (self.stuck_check - 5) * 0.01
                    self.get_logger().info(
                        f"Stuck. Increasing damping factor to {stuck_parameter}"
                    )
                w = self.p_rotation_one * final_dtheta
                v = 0

        # Damping
        v = v * self.v_damper
        w = w * self.w_damper

        if abs(v) > 1.5:
            v = np.sign(v) * 1.5
        if abs(w) > 1.5:
            w = np.sign(w) * 1.5

        # Update last values
        self.last_distance = distance
        self.last_delta_theta = delta_theta

        # Convert to duty cycles
        motor_msg = DutyCycles()
        motor_msg.duty_cycle_left = (v - 0.5 * w) * (
            self.cycle_damping + stuck_parameter
        )
        motor_msg.duty_cycle_right = (v + 0.5 * w) * (
            self.cycle_damping + stuck_parameter
        )
        motor_msg.header.stamp = self.get_clock().now().to_msg()

        # Logging
        # self.get_logger().info(f"v: {v}, w: {w}, distance: {distance}")

        # Publish the duty cycles
        self.motion_publisher.publish(motor_msg)
        return

    def stop(self):
        motor_msg = DutyCycles()
        motor_msg.duty_cycle_left = 0
        motor_msg.duty_cycle_right = 0
        motor_msg.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info("Robot stopped")

    def notify_reached_destination(self, success):
        msg = Bool()
        msg.data = success
        self.reached_destination_publisher.publish(msg)

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

    def check_lidar(self):
        # TODO: Implement lidar check
        # Check if an obstacle is detected
        if False:
            self.obstacle_detected = True
            self.get_logger().info("Obstacle detected")
            self.stop_robot()
            return
        pass


def main(args=None):
    rclpy.init(args=args)

    try:
        motion_controller = MotionController()
        motion_controller.get_logger().info("MotionController node has started.")

        rclpy.spin(motion_controller)
    except KeyboardInterrupt:
        motion_controller.get_logger().info("MotionController node is shutting down.")
    finally:
        motion_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()