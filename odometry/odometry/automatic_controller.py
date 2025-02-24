#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped, Twist
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from robp_interfaces.msg import DutyCycles

import tf2_geometry_msgs


class Controller(Node):
    """
    Directs the robot iteratively towards a position defined in topic /path/nextpos (TransformStamped)
    needs continous republishing of the goal position in topic
    """

    def __init__(self):
        super().__init__("path_follower")  # Call the superclass constructor
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.create_subscription(
            TransformStamped, "/path/nextpos", self.nextpos_callback, 10
        )
        # Initialize the publisher for motor duty cycles
        self._pub = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)
        self.goal_position = TransformStamped()

        self.goal_margin_translational = 0.05
        self.goal_margin_rotational = math.pi / 6

        self.speed_factor = 0,5

        self.speed_lin = 1.5
        self.speed_rot = 0.75

        self.p_rotation = (
            12  # 0 !< p_rotation !< 2base/(h*radius) =  12.599/h h:=sampling time
        )
        self.p_translation = (
            30  # 0 !< p_translation !< 2/(h*radius) = 40.642/h h:=sampling time
        )

    def nextpos_callback(self, msg: TransformStamped):
        self.goal_position = msg
        # init
        time = msg.header.stamp
        robot_frame = msg.child_frame_id
        goal_frame = "base_link"

        goal_margin_translational = 0.05
        goal_margin_rotational = math.pi / 10

        # cmd_vel = Twist()
        # cmd_vel.linear.x = 0.0
        # cmd_vel.angular.z = 0.0

        # Wait for the transform asynchronously
        compared_transform = self.buffer.wait_for_transform_async(
            target_frame=goal_frame, source_frame=robot_frame, time=time
        )
        rclpy.spin_until_future_complete(self, compared_transform, timeout_sec=0.5)

        # Check if the future completed successfully
        if not compared_transform.done():
            self.get_logger().error(
                f"Transform future did not complete successfully between {robot_frame} and {goal_frame}."
            )
            return
        else:
            finished_transform = compared_transform.result()
            comp_translation = finished_transform.transform.translation
            comp_rotation = finished_transform.transform.rotation
            distance_to_point = math.sqrt(comp_translation.x**2 + comp_translation.y**2)

            alpha, beta, theta_goal = euler_from_quaternion([comp_rotation.x, comp_rotation.y, comp_rotation.z, comp_rotation.w])
            theta_dir = math.atan2(comp_translation.y, comp_translation.x)
            delta_theta = theta_dir
            print(delta_theta)

        try:
            if abs(delta_theta) > goal_margin_rotational:
                w = self.p_rotation * delta_theta
                v = 0
                # v = self.p_trans_rotation * (cos(theta_))
                print(f"Rotating towards {[comp_translation.x, comp_translation.y]}")
                print(f"Rotating w: {w}")
            elif distance_to_point > goal_margin_translational:
                v = self.p_translation * (
                    comp_translation.x * math.cos(delta_theta)
                    + comp_translation.y * math.sin(delta_theta)
                )
                # w = self.p_rot_translation * (math.sin(delta_theta*))
                w = 0
                print(f"Moving towards {[comp_translation.x, comp_translation.y]}")
                print(f"v: {v}")
            elif (distance_to_point < goal_margin_translational) and (
                theta_goal > goal_margin_rotational
            ):
                # w = self.p_rotation*theta_goal
                print(f"AT GOAL\nROTATING DIST: {theta_goal}")
            print(f"v: {v}, w: {w}")

            # Damping
            v_damper = 1
            w_damper = 1
            v = v * v_damper
            w = w * w_damper

            if abs(v) > 1:
                v = np.sign(v)*1.5
            if abs(w) > 1:
                w = np.sign(w)
            # Convert to duty cycles
            motor_msg = DutyCycles()
            motor_msg.duty_cycle_left = (v - 0.5 * w)*0.1
            motor_msg.duty_cycle_right = (v + 0.5 * w)*0.1

            motor_msg.header.stamp = self.get_clock().now().to_msg() # msg.header.stamp

            # Publish the message
            self._pub.publish(motor_msg)
            # self.get_logger().info(f"Published: {motor_msg.duty_cycle_left}, {motor_msg.duty_cycle_right}")
            # print(self._pub.topic)
            return

        except Exception as ex:
            # Log any errors (this will only log broadcasting issues now)
            self.get_logger().error(
                f"Failed to move towards position: {msg.transform} \n {ex}"
            )
            return


def main():
    rclpy.init()
    node = Controller()
    # try:
    #     while rclpy.ok():
    #         rclpy.spin_once(node)
    #         node.do_go_to_point()
    # except KeyboardInterrupt:
    #     pass
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
