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
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.create_subscription(
            TransformStamped, "/path/nextpos", self.nextpos_callback, 10
        )
        self.publisher = self.createPublisher(Twist, "/cmd_vel", 10)

    def nextpos_callback(self, msg: TransformStamped):
        # init
        time = msg.header.stamp
        robot_frame = "base_link"
        goal_frame = msg.child_frame_id

        goal_margin_translational = 0.05
        goal_margin_rotational = math.pi / 10

        cmd_vel = Twist()

        # # Wait for the transform asynchronously
        compared_transform = self.tf_buffer.wait_for_transform_async(
            target_frame=goal_frame, source_frame=robot_frame, time=time
        )
        rclpy.spin_until_future_complete(self, compared_transform, timeout_sec=0.5)

        comp_translation = compared_transform.transform.translation
        comp_rotation = comp_translation.transform.rotation

        # Check if the future completed successfully
        if not compared_transform.done():
            self.get_logger().error(
                f"Transform future did not complete successfully between {robot_frame} and {goal_frame}."
            )
            return
        try:
            if (abs(comp_translation.y) < goal_margin_translational) & (
                abs(comp_translation.x) < goal_margin_translational
            ):
                if abs(comp_rotation.z) < math.pi / 8:
                    cmd_vel.linear.x = 0
                    cmd_vel.angular.z = 0
                elif comp_rotation.z > 0:
                    cmd_vel.angular.z = 1.5
                elif comp_rotation.z < 0:
                    cmd_vel.angular.z = -1.5

            elif (abs(comp_translation.y) < goal_margin_translational) & (
                comp_translation.x > 0
            ):
                # go forward
                cmd_vel.linear.x = 1.5
                cmd_vel.angular.z = 0

            else:
                if comp_translation.y > 0:
                    cmd_vel.angular.z = 1.5
                elif comp_translation.y < 0:
                    cmd_vel.angular.z = -1.5
                cmd_vel.linear.x = 0

            self.publisher.publish(cmd_vel)

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
