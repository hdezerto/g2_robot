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

import tf2_geometry_msgs


class pathFollowerNode(Node):
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
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

    def nextpos_callback(self, msg: TransformStamped):
        # init
        time = msg.header.stamp
        robot_frame = msg.child_frame_id
        goal_frame = "base_link"

        goal_margin_translational = 0.05
        goal_margin_rotational = math.pi / 10
        speed_lin = 1.5
        speed_rot = 1.5
        margin_rotate_first = 0.2  # for points below this distance the robot will rotate first without going forward

        cmd_vel = Twist()

        # # Wait for the transform asynchronously
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

        try:
            # check whether it is already at goal point
            if (abs(comp_translation.y) < goal_margin_translational) and (
                abs(comp_translation.x) < goal_margin_translational
            ):
                print("Already at goal point")
                # check whether orientation is correct, rotate if not
                if abs(comp_rotation.z) < goal_margin_rotational:
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 0.0
                    self.get_logger().info("Position already reached.")
                elif comp_rotation.z > 0.0:
                    cmd_vel.angular.z = speed_rot
                elif comp_rotation.z < 0.0:
                    cmd_vel.angular.z = -speed_rot
            # # check whether the robot is oriented to move towards the point
            # elif (abs(comp_translation.y) < goal_margin_translational) and (
            #     comp_translation.x > 0
            # ):
            #     # go forward
            #     cmd_vel.linear.x = speed_lin
            #     cmd_vel.angular.z = 0

            # if not oriented
            else:
                if comp_translation.y > 0.0:
                    if abs(comp_translation.y) < goal_margin_translational:
                        cmd_vel.angular.z = speed_rot
                        print("turning left")
                    if (distance_to_point > margin_rotate_first) and (
                        comp_translation.x > 0.0
                    ):
                        cmd_vel.linear.x = speed_lin
                        print("move ahead")

                elif comp_translation.y < 0.0:
                    if abs(comp_translation.y) < goal_margin_translational:
                        cmd_vel.angular.z = -speed_rot
                        print("turning right")
                    if (distance_to_point > margin_rotate_first) and (
                        comp_translation.x > 0.0
                    ):
                        cmd_vel.linear.x = speed_lin
                        print("move ahead")

                # cmd_vel.linear.x = 0.0
            # print(f"Message \n{msg}")
            # print(f"compare transform \n{[comp_translation.x, comp_translation.y]}")
            # print([cmd_vel.linear.x, cmd_vel.angular.z])

            # cmd_vel.linear.x = 100.0
            self.publisher.publish(cmd_vel)
            self.get_logger().info("Published!")

            return
        except Exception as ex:
            # Log any errors (this will only log broadcasting issues now)
            self.get_logger().error(
                f"Failed to move towards position: {msg.transform} \n {ex}"
            )
            return


def main():
    rclpy.init()
    node = pathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
