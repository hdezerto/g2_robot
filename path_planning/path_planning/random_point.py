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

from geometry_msgs.msg import Pose
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from ament_index_python.packages import get_package_share_directory


from shapely.geometry import Polygon, Point
from builtin_interfaces.msg import Duration


import tf2_geometry_msgs


class RandomWsPoint(Node):
    """ """

    def __init__(self):
        super().__init__("random_point")  # Call the superclass constructor

        workspace_filename = "workspace_2.tsv"
        package_share_directory = get_package_share_directory("path_planning")
        self.ws_file = os.path.join(
            package_share_directory, "resource", workspace_filename
        )
        self.ws_vertices = self.read_tsv()
        self.ws_polygon = Polygon(self.ws_vertices)
        self.ws_polygon_buffered = self.ws_polygon.buffer(-20)

        self.srv = self.create_service(
            Pose, "get_random_ws_point", self.get_new_point_callback
        )

    def get_new_point_callback(self, request, response):
        """
        generate new point

        assign random point in workspace (within a margin) to self.goal_position.transform.translation.x/y
        #/ assign random orientation within [0, 2pi] as quaternion to goal_position.transform.rotation.z

        """
        # # generate new point
        # random_x = random.uniform(
        #     -(self.workspace[0] - 0.10) / 2, (self.workspace[0] - 0.10) / 2
        # )
        # random_y = random.uniform(
        #     -(self.workspace[1] - 0.10) / 2, (self.workspace[1] - 0.10) / 2
        # )
        random_x, random_y = self.point_in_ws()
        print(random_x, random_y)
        random_rot = random.uniform(0, 2 * math.pi)
        response.position.x = random_x
        response.position.y = random_y
        response.orientation = quaternion_from_euler(0, 0, random_rot)

        return response

    def read_tsv(self):
        vertices = []
        with open(self.ws_file, mode="r") as file:
            reader = csv.reader(file, delimiter="\t")
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
                return (x / 100, y / 100)  # cm to meters


def main():
    rclpy.init()
    node = RandomWsPoint()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
