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


class pathPublisherNode(Node):
    """ """

    def __init__(self):
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.publisher = self.createPublisher(TransformStamped, "/path/nextpos", 10)

        self.position_reached = True
        self.goal_position = TransformStamped()

        self.workspace = [100, 100]

    def go_to_point(self):
        """
        publish transform to topic
        """
        # init
        self.goal_position.header.stamp = self.get_clock().now()
        time = self.goal_position.header.stamp
        robot_frame = "base_link"
        goal_frame = self.goal_position.child_frame_id
        goal_margin_translational = 0.05
        goal_margin_rotational = math.pi / 10

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
                f"Transform future did not complete successfully for Ball"
            )
            return
        try:
            if (
                (abs(comp_translation.x) < goal_margin_translational)
                & (abs(comp_translation.y) < goal_margin_translational)
                & (abs(comp_rotation.z) < goal_margin_rotational)
            ):
                self.position_reached = True
                self.get_logger().info(
                    f"Position {self.goal_position.transform} has been reached!"
                )
            else:
                self.publisher.publish(self.goal_position)

            return
        except Exception as ex:
            # Log any errors (this will only log broadcasting issues now)
            self.get_logger().error(f"Failed to process ball Error: {ex}")
            return

    def get_new_point(self):
        """
        service request new point
        """
        self.position_reached = False
        # assign random point in workspace (within a margin) to self.goal_position.transform.translation.x/y


def main():
    rclpy.init()
    node = pathPublisherNode()

    try:
        while rclpy.ok():
            if node.update_goal_position:
                node.get_new_point()
            node.go_to_point()

    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
