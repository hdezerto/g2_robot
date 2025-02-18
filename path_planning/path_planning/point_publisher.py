#!/usr/bin/env python

import math

import random
import numpy as np

import os
import csv

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

from ament_index_python.packages import get_package_share_directory
from shapely.geometry import Polygon, Point

import tf2_geometry_msgs


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

        workspace_filename = "workspace_1.tsv"
        package_share_directory = get_package_share_directory("path_planning")
        self.ws_file = os.path.join(
            package_share_directory, "resource", workspace_filename
        )
        self.ws_vertices = self.read_tsv()
        self.ws_polygon = Polygon(self.ws_vertices)
        self.ws_polygon_buffered = self.ws_polygon.buffer(-20)

    def go_to_point(self):
        """
        publish transform to topic
        """
        # print(f"GOING TOWARDS: {[self.goal_position.transform.translation.x, self.goal_position.transform.translation.y, self.goal_position.transform.rotation.z]}")
        
        self.tf_broadcaster.sendTransform(self.goal_position)

        # init
        goal_transform = self.goal_position
        # goal_transform.header.stamp = self.get_clock().now().to_msg()
        # time = self.get_clock().now().to_msg()
        time = self.goal_position.header.stamp
        robot_frame = "base_link"
        goal_frame = goal_transform.child_frame_id
        goal_margin_translational = 0.05
        goal_margin_rotational = math.pi / 10

        # broadcast transform
        self.tf_broadcaster.sendTransform(goal_transform)

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
        if not (compared_transform.done()):  # and compared_transform.result()
            self.get_logger().error(
                f"Transform future did not complete successfully for time {time}"
            )
            return
        # else:
            # self.tf_broadcaster.sendTransform(self.goal_position)

        try:
            # transform translation and rotation
            # print(type(compared_transform.result()))
            finished_transform = compared_transform.result()
            # print(transform)
            comp_translation = finished_transform.transform.translation
            comp_rotation = finished_transform.transform.rotation
            distance_to_point = math.sqrt(comp_translation.x**2 + comp_translation.y**2)

            if (
                distance_to_point < goal_margin_translational * 2
            ):  # and (abs(comp_rotation.z) < goal_margin_rotational)
                self.position_reached = True
                self.get_logger().info(
                    f"Position {[goal_transform.transform.translation.x, goal_transform.transform.translation.y, goal_transform.transform.rotation.z]} has been reached!"
                )
                self.goal_position = self.get_new_point()
                print(
                    f"NEW GOAL POSITION {[self.goal_position.transform.translation.x, self.goal_position.transform.translation.y, self.goal_position.transform.rotation.z]}"
                )
            else:
                # self.tf_broadcaster.sendTransform(self.goal_position)
                self.publisher.publish(goal_transform)
                

            return
        except Exception as ex:
            # Log any errors (this will only log broadcasting issues now)
            self.get_logger().error(f"Error: {ex}")
            return

    def get_new_point(self):
        """
        generate new point

        assign random point in workspace (within a margin) to self.goal_position.transform.translation.x/y
        assign random orientation within [0, 2pi] as quaternion to goal_position.transform.rotation.z

        """
        self.position_reached = False

        # # generate new point
        # random_x = random.uniform(
        #     -(self.workspace[0] - 0.10) / 2, (self.workspace[0] - 0.10) / 2
        # )
        # random_y = random.uniform(
        #     -(self.workspace[1] - 0.10) / 2, (self.workspace[1] - 0.10) / 2
        # )
        (random_x, random_y) = self.point_in_ws()
        print(random_x, random_y)
        random_rot = random.uniform(0, 2 * math.pi)

        # initialize new transform
        goal_transform = TransformStamped()
        goal_transform.header.frame_id = "map"
        goal_transform.child_frame_id = "goal_position"
        goal_transform.header.stamp = self.get_clock().now().to_msg()
        # print(type(self.get_clock().now().to_msg()))

        # assign random point to transform
        goal_transform.transform.translation.x = random_x
        goal_transform.transform.translation.y = random_y
        random_quaternion = quaternion_from_euler(0, 0, random_rot)
        goal_transform.transform.rotation.z = random_quaternion[2]
        goal_transform.transform.rotation.w = random_quaternion[3]

        
        self.goal_position = goal_transform
        # goal_transform_static = goal_transform
        # goal_transform_static.child_frame_id = "goal_position_static"
        # self.static.sendTransform(goal_transform)
        # self.tf_broadcaster.sendTransform(goal_transform)
        # self.tf_broadcaster.sendTransform(goal_transform)
        print(f"GOT NEW POINT:\n{[goal_transform.transform.translation.x, goal_transform.transform.translation.y, goal_transform.transform.rotation.z]}")
        print(
            f"GOT NEW POINT(self):\n{[self.goal_position.transform.translation.x, self.goal_position.transform.translation.y, self.goal_position.transform.rotation.z]}"
        )

        return self.goal_position

    def do_broadcast(self):
        
        goal_transform = self.goal_position
        self.goal_position.header.stamp = self.get_clock().now().to_msg()
        goal_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(goal_transform)

    def read_tsv(self):
        vertices = []
        with open(self.ws_file, mode='r') as file:
            reader = csv.reader(file, delimiter='\t')
            next(reader)  # Skip header row
            for row in reader:
                x, y = float(row[0]), float(row[1])
                vertices.append((x, y))
        return vertices
    
    def point_in_ws(self):
        min_x, min_y, max_x, max_y = self.ws_polygon_buffered.bounds
        while True:
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            point = Point(x, y)
            if self.ws_polygon_buffered.contains(point):
                return (x/100, y/100) # cm to meters
    
    def


def main():
    rclpy.init()
    node = pathPublisherNode()

    try:
        node.goal_position = node.get_new_point()
        while rclpy.ok():
            node.do_broadcast()
            rclpy.spin_once(node)
            node.go_to_point()
            

    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
