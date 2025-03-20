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
from data_types.srv import RandomPoint

from ament_index_python.packages import get_package_share_directory

from enum import Enum

import tf2_geometry_msgs

class GoalService(Node):
    def __init__(self):
        super().__init__("goal_service")

        package_share_directory = get_package_share_directory("path_planning")
        self.ws_file = os.path.join(
            package_share_directory, "resource", "map_1.tsv"
        )
        self.objects, self.boxes = self.read_tsv(self.ws_file)        
        
        self.srv_object = self.create_service(RandomPoint, 'new_object_pos', self.new_object_pos_callback)


    def new_object_pos_callback(self):
        current_x, current_y = self.get_current_position()
        
        shortest_dist = 3000
        obj_index = 0
        for index, object in enumerate(self.objects):
            obj_x, obj_y = object[0], object[1]
            distance = math.sqrt((current_x-obj_x)^2 + (current_y - obj_y)^2)
            if distance < shortest_dist:
                shortest_dist = distance
                obj_index = index

    def get_current_position(self):
        time = self.self.get_clock().now().to_msg()
        # init
        goal_transform = self.goal_position
        robot_frame = "map"
        goal_frame = "base_link"

        # Wait for the transform asynchronously
        current_position_future = self.buffer.wait_for_transform_async(
            target_frame=robot_frame, source_frame=goal_frame, time=time
        )

        rclpy.spin_until_future_complete(self, current_position_future, timeout_sec=2)
        current_x = 0
        current_y = 0
        # Check if the future completed successfully
        if not (current_position_future.done()):  # and compared_transform.result()
            self.get_logger().error(
                f"Transform future did not complete successfully for time {time}"
            )
            return
        else:
            current_position = current_position_future.result()
            current_x = current_position.transform.translation.x
            current_y = current_position.transform.translation.y
        return current_x, current_y


    def read_tsv(self, file_path):
        objects = []
        boxes = []
        with open(file_path, mode="r") as file:
            reader = csv.reader(file, delimiter="\t")
            next(reader)  # Skip header row
            for row in reader:
                x, y, theta = (
                    float(row[1]) / 100,
                    float(row[2]) / 100,
                    float(row[3]) * math.pi / 180,
                )
                if row[0] == "B":
                    boxes.append((x, y, theta))
                else:
                    objects.append((x, y, theta))

        return objects, boxes