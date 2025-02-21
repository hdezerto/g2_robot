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
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

from ament_index_python.packages import get_package_share_directory
from data_types.srv import RandomPoint

from heapq import heappush, heappop

import tf2_geometry_msgs


class PathPlanningNode(Node):
    """
    Limitations: Boxes have to be oriented a certain way.
    """

    def __init__(self):
        super().__init__("path_planning")

        self.start = (0, 0)
        self.goal = (0, 0)
        self.goal_positions = []
        self.object_position = RandomPoint()
        self.box_position = RandomPoint()

        self.create_subscription(OccupancyGrid, "map_update", self.map_update, 10)
        self.create_subscription(RandomPoint, "new_object_pos", self.path_to_object, 10)
        self.create_subscription(RandomPoint, "new_box_pos", self.path_to_box, 10)

        self.path_publisher = self.create_publisher(Path, "/path/planned_path")

    def path_to_object(self, msg: RandomPoint):
        print("TODO")
        self.start = self.get_current_position()
        self.object_position = msg
        self.goal_positions = offsets
        self.find_path(offsets)

    def path_to_box(self, msg: RandomPoint):
        print("TODO")
        current_pos = self.get_current_position()
        self.start = (
            int(current_pos[0] / self.map_resolution),
            int(current_pos[1] / current_pos[1] / self.map_resolution),
        )
        self.box_position = msg
        self.goal = (0, 0)  # TOTO
        # define goal states

    def get_goal_positions(self, radius):
        gx, gy = self.goal
        offsets = [
            (gx, gy + int(self.offset_distance / self.map_resolution)),  # North
            (gx - int(self.offset_distance / self.map_resolution), gy),  # West
            (gx, gy - int(self.offset_distance / self.map_resolution)),  # South
            (gx + int(self.offset_distance / self.map_resolution), gy),  # East
        ]
        return offsets

    def find_path(self, goals):
        print("TODO")
        # find path to goal states
        path = self.astar(self.start, self.goal_positions)
        if path:
            self.publish_path(path)

    def astar(self, start, goals):
        print("TODO")
        open_list = []
        closed_list = set()

        start_node = (0, self.start, None)
        heappush(open_list, start_node)

        while open_list:
            current_node = heappop(open_list)
            current_f, current_position, parent = current_node

            if current_position in closed_list:
                continue

            closed_list.add(current_position)

            if current_position in self.goal_positions:
                path = []
                while current_node:
                    path.append(current_node[1])
                    current_node[1] = current_node[2]
                return path[::-1]

            neighbours = self.get_neighbours(current_position)
            for neighbour in neighbours:
                if neighbour in closed_list:
                    continue

                g = current_f + 1
                h = self.heuristic(neighbour, goal)
                f = g + h
                neighbour_node = (f, neighbour, current_node)

                heappush(open_list, neighbour_node)

        return None

    def map_update(self, msg):
        print("TODO")
        self.map_data = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def get_current_position(self) -> float:
        time = self.self.get_clock().now().to_msg()

        # Wait for the transform asynchronously
        current_position_future = self.buffer.wait_for_transform_async(
            target_frame="map", source_frame="base_link", time=time
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

    def get_neighbours(self, position):
        x, y = position
        neighbours = []
        for dx, dy in [(-1, 0), (1, 0), (0, 1), (0, -1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                if self.is_free(nx, ny):
                    neighbours.append((nx, ny))
        return neighbours

    def is_free(self, x, y):
        index = y * self.map_width + x
        return self.map_data[index] == 0

    def heuristic(self, pos, goal):
        return math.sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for x, y in path:
            pose = PoseStamped()
            pose.pose.position.x = x * self.map_resolution + self.map_origin[0]
            pose.pose.position.y = y * self.map_resolution + self.map_origin[1]
            path_msg.poses.append(pose)

        final_pose = PoseStamped()
        final_pose.pose.position.x = (
            best_offset_point[0] * self.map_resolution + self.map_origin[0]
        )
        final_pose.pose.position.y = (
            best_offset_point[1] * self.map_resolution + self.map_origin[1]
        )
        angle_to_goal = atan2(
            self.goal[1] - best_offset_point[1], self.goal[0] - best_offset_point[0]
        )
        final_pose.pose.orientation.z = sin(angle_to_goal / 2.0)
        final_pose.pose.orientation.w = cos(angle_to_goal / 2.0)
        path_msg.poses.append(final_pose)

        self.path_publisher.publish(path_msg)


def main():
    rclpy.init()
    node = PathPlanningNode()
    rclpy.spin(node)
    # try:
    #     node.goal_position = node.get_new_point()
    #     while rclpy.ok():
    #         node.do_broadcast()
    #         rclpy.spin_once(node)
    #         node.go_to_point()

    # except KeyboardInterrupt:
    #     pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
