#!/usr/bin/env python

import math

import random
import numpy as np

import rclpy
from rclpy.node import Node

# from builtin_interfaces.msg import Time

import rclpy.time
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped, Twist
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import tf2_geometry_msgs


# class pathPublisherNode(Node):
#     """ """

#     def __init__(self):
#         super().__init__("point_publisher")  # Call the superclass constructor
#         self.buffer = Buffer()
#         self.listener = TransformListener(self.buffer, self, spin_thread=True)
#         self.publisher = self.create_publisher(TransformStamped, "/path/nextpos", 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.position_reached = True
#         self.goal_position = TransformStamped()

#         self.workspace = [1.0, 1.0]

#     def go_to_point(self):
#         """
#         publish transform to topic
#         """
#         # init
#         goal_transform = self.goal_position
#         goal_transform.header.stamp = self.get_clock().now().to_msg()
#         time = self.get_clock().now().to_msg()
#         robot_frame = "base_link"
#         goal_frame = goal_transform.child_frame_id # "goal_position"
#         goal_margin_translational = 0.05
#         goal_margin_rotational = math.pi / 10

#         # broadcast transform
#         self.tf_broadcaster.sendTransform(goal_transform)

#         # Wait for the transform synchronously
#         try:
#             compared_transform = self.buffer.lookup_transform(
#                 target_frame=robot_frame, source_frame=goal_frame, time=time, timeout=rclpy.duration.Duration(seconds=2)
#             )
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
#             self.get_logger().error(f"Transform lookup failed: {ex}")
#             return

#         # transform translation and rotation
#         comp_translation = compared_transform.transform.translation
#         comp_rotation = compared_transform.transform.rotation
#         if (
#             (abs(comp_translation.x) < goal_margin_translational)
#             and (abs(comp_translation.y) < goal_margin_translational)
#             and (abs(comp_rotation.z) < goal_margin_rotational)
#         ):
#             self.position_reached = True
#             self.get_logger().info(
#                 f"Position {goal_transform.transform} has been reached!"
#             )
#         else:
#             self.publisher.publish(goal_transform)

#     def get_new_point(self):
#         """
#         generate new point

#         assign random point in workspace (within a margin) to self.goal_position.transform.translation.x/y
#         assign random orientation within [0, 2pi] as quaternion to goal_position.transform.rotation.z

#         """
#         self.position_reached = False

#         # generate new point
#         random_x = random.uniform(
#             -(self.workspace[0] - 10) / 2, (self.workspace[0] - 10) / 2
#         )
#         random_y = random.uniform(
#             -(self.workspace[0] - 10) / 2, (self.workspace[0] - 10) / 2
#         )
#         random_rot = random.uniform(0, 2 * math.pi)

#         # initialize new transform
#         self.goal_position = TransformStamped()
#         self.goal_position.header.frame_id = "map"
#         self.goal_position.child_frame_id = "goal_position"
#         self.goal_position.header.stamp = self.get_clock().now().to_msg()
#         # print(type(self.get_clock().now().to_msg()))

#         # assign random point to transform
#         self.goal_position.transform.translation.x = random_x
#         self.goal_position.transform.translation.y = random_y
#         random_quaternion = quaternion_from_euler(0, 0, random_rot)
#         self.goal_position.transform.rotation.z = random_quaternion[2]
#         self.goal_position.transform.rotation.w = random_quaternion[3]

#         self.get_logger().info(
#             f"New point:\n{[self.goal_position.transform.translation.x, self.goal_position.transform.translation.y, self.goal_position.transform.translation.z]}\n{[self.goal_position.transform.rotation.x, self.goal_position.transform.rotation.y, self.goal_position.transform.rotation.z, self.goal_position.transform.rotation.w]}"
#         )


# def main():
#     rclpy.init()
#     node = pathPublisherNode()

#     try:
#         while rclpy.ok():
#             if node.position_reached:
#                 node.get_new_point()
#             node.go_to_point()

#     except KeyboardInterrupt:
#         pass
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


class pathPublisherNode(Node):
    """ """

    def __init__(self):
        super().__init__("point_publisher")  # Call the superclass constructor
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.publisher = self.create_publisher(TransformStamped, "/path/nextpos", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)

        self.position_reached = True
        self.goal_position = TransformStamped()

        self.workspace = [1.0, 1.0]

    def go_to_point(self):
        """
        publish transform to topic
        """
        # init
        goal_transform = self.goal_position
        goal_transform.header.stamp = self.get_clock().now().to_msg()
        time = self.get_clock().now().to_msg()
        robot_frame = "base_link"
        goal_frame = goal_transform.child_frame_id
        goal_margin_translational = 0.05
        goal_margin_rotational = math.pi / 10

        # broadcast transform
        # self.tf_broadcaster.sendTransform(goal_transform)

        # Wait for the transform asynchronously
        compared_transform = self.buffer.wait_for_transform_async(
            target_frame=robot_frame, source_frame=goal_frame, time=time
        )

        rclpy.spin_until_future_complete(self, compared_transform, timeout_sec=2)

        # try:
        #     # print(compared_transform.result().transform)
        # except rclpy.executors.TimeoutException:
        #     self.get_logger().error(f"Transform future did not complete successfully for time {time}")
        # except Exception as ex:
        #     self.get_logger().error(f"An error occurred: {ex}")


        # Check if the future completed successfully
        if not (compared_transform.done()): # and compared_transform.result()
            self.get_logger().error(
                f"Transform future did not complete successfully for time {time}"
            )
            return
        
        try:
            # transform translation and rotation
            # print(type(compared_transform.result()))
            finished_transform = compared_transform.result()
            # print(transform)
            comp_translation = finished_transform.transform.translation
            comp_rotation = finished_transform.transform.rotation
            distance_to_point = math.sqrt(comp_translation.x**2 + comp_translation.y**2)

            if (
                distance_to_point < goal_margin_translational*2
            ): # and (abs(comp_rotation.z) < goal_margin_rotational)
                self.position_reached = True
                self.get_logger().info(
                    f"Position {goal_transform.transform} has been reached!"
                )
            else:
                self.publisher.publish(goal_transform)

            return
        except Exception as ex:
            # Log any errors (this will only log broadcasting issues now)
            self.get_logger().error(f"Failed to process ball Error: {ex}")
            return

    def get_new_point(self):
        """
        generate new point

        assign random point in workspace (within a margin) to self.goal_position.transform.translation.x/y
        assign random orientation within [0, 2pi] as quaternion to goal_position.transform.rotation.z

        """
        self.position_reached = False

        # generate new point
        random_x = random.uniform(
            -(self.workspace[0] - 0.10) / 2, (self.workspace[0] - 0.10) / 2
        )
        random_y = random.uniform(
            -(self.workspace[1] - 0.10) / 2, (self.workspace[1] - 0.10) / 2
        )
        print(random_x, random_y)
        random_rot = random.uniform(0, 2 * math.pi)

        # initialize new transform
        self.goal_position = TransformStamped()
        self.goal_position.header.frame_id = "map"
        self.goal_position.child_frame_id = "goal_position"
        self.goal_position.header.stamp = self.get_clock().now().to_msg()
        # print(type(self.get_clock().now().to_msg()))

        # assign random point to transform
        self.goal_position.transform.translation.x = random_x
        self.goal_position.transform.translation.y = random_y
        random_quaternion = quaternion_from_euler(0, 0, random_rot)
        self.goal_position.transform.rotation.z = random_quaternion[2]
        self.goal_position.transform.rotation.w = random_quaternion[3]

        self.get_logger().info(
            f"New point:\n{[self.goal_position.transform.translation.x, self.goal_position.transform.translation.y, self.goal_position.transform.translation.z]}\n{[self.goal_position.transform.rotation.x, self.goal_position.transform.rotation.y, self.goal_position.transform.rotation.z, self.goal_position.transform.rotation.w]}"
        )

        self.static.sendTransform(self.goal_position)


def main():
    rclpy.init()
    node = pathPublisherNode()

    try:
        while rclpy.ok():
            if node.position_reached:
                node.get_new_point()
            node.go_to_point()

    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()


