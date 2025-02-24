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
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid

from heapq import heappush, heappop


class PathPlanningNode(Node):
    """
    Limitations: Boxes have to be oriented a certain way.
    """

    def __init__(self):
        super().__init__("path_planning")

        self.start_point = (0, 0)
        self.start_map = (0, 0)
        self.goal_map = (0, 0)
        self.goal_point = (0, 0)
        self.goal_positions = []
        self.object_position = Pose()
        self.box_position = Pose()

        self.use_bu = True
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)

        self.create_subscription(OccupancyGrid, "map_update", self.map_update, 10)
        self.create_subscription(Pose, "new_object_pos", self.path_to_object, 10)
        self.create_subscription(Pose, "new_box_pos", self.path_to_box, 10)

        self.path_publisher = self.create_publisher(Path, "/path/planned_path", 10)
        self.backup_publisher = self.create_publisher(TransformStamped, "/path/goal", 10)


    def path_to_object(self, msg: Pose):
        print("TODO")
        current_pos = None
        while not current_pos:
            current_pos = self.get_current_position()
        self.object_position = msg
        self.goal_point = (msg.position.x, msg.position.y)
        
        if not self.use_bu:
            self.goal_map = (
                int(msg.position.x / self.map_resolution),
                int(msg.position.y / self.map_resolution),
            )
            self.goal_positions = self.get_goal_positions(0.2)
            self.find_path(self.goal_positions, 0.2)
        else:
            self.backup_planning((msg.position.x, msg.position.y), 0.2)

    def path_to_box(self, msg: Pose):
        print("TODO")
        current_pos = None
        while not current_pos:
            current_pos = self.get_current_position()
        # self.start_map = (
        #     int(current_pos[0] / self.map_resolution),
        #     int(current_pos[1] / self.map_resolution),
        # )
        # self.start_point = current_pos
        self.box_position = msg
        self.goal_point = (msg.position.x, msg.position.y)
        # self.goal = (0, 0)  # TOTOfgoa
        if not self.use_bu:
            self.goal_map = (
                int(msg.position.x / self.map_resolution),
                int(msg.position.y / self.map_resolution),
            )
            self.goal_positions = self.get_goal_positions(0.2)
            self.find_path(self.goal_positions, 0.2)
        else:
            self.backup_planning((msg.position.x, msg.position.y), 0.2)

    def get_goal_positions(self, radius):
        gx, gy = self.goal_map[0], self.goal_map[1]
        offsets = [
            (gx, gy + int(radius / self.map_resolution), "n"),  # North
            (gx - int(radius / self.map_resolution), gy),
            "w",  # West
            (gx, gy - int(radius / self.map_resolution)),
            "s",  # South
            (gx + int(radius / self.map_resolution), gy, "e"),  # East
        ]
        goal_positions = []
        for offset in offsets:
            if self.is_free(offset[0], offset[1]):
                goal_positions.append(offset)
        if goal_positions:
            return goal_positions
        else:
            self.get_logger().error("No possible goal positions.")

    def find_path(self, goals, radius):
        print("TODO")
        smallest_f = float('inf')
        goal_path = None
        direction = ""
        # find path to goal states
        for goal_pos in goals:
            path, f = self.astar(self.start_map, (goal_pos[0], goal_pos[1]))
            if path and f < smallest_f:
                goal_path = path
                smallest_f = f
                direction = goal_pos[2]
        if goal_path:
            self.publish_path(goal_path, direction, radius)

    def astar(self, start: tuple[int, int], goal: tuple[int, int]):
        print("TODO")
        open_list = []
        closed_list = set()

        start_node = (0, start, None)
        heappush(open_list, start_node)

        while open_list:
            current_node = heappop(open_list)
            current_f, current_position, parent = current_node

            if current_position in closed_list:
                continue

            closed_list.add(current_position)

            if current_position == goal:
                path = []
                while current_node:
                    path.append(current_node[1])
                    current_node[1] = current_node[2]
                return path[::-1], current_f

            neighbours = self.get_neighbours(current_position)
            for neighbour in neighbours:
                if neighbour in closed_list:
                    continue

                g = current_f + 1
                h = self.heuristic(neighbour, goal)
                f = g + h
                neighbour_node = (f, neighbour, current_node)

                heappush(open_list, neighbour_node)

        return None, 1000000

    def map_update(self, msg):
        # print("TODO")
        self.map_data = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def get_current_position(self) -> float:
        time = self.get_clock().now().to_msg()

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

        self.start_point = (current_x, current_y)
        if not self.use_bu:
            self.start_map = (
                int(current_x / self.map_resolution),
                int(current_y / self.map_resolution),
            )
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

    def publish_path(self, path, direction, radius):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for x, y in path:
            pose = PoseStamped()
            pose.pose.position.x = x * self.map_resolution + self.map_origin[0]
            pose.pose.position.y = y * self.map_resolution + self.map_origin[1]
            path_msg.poses.append(pose)

        final_pose = PoseStamped()
        final_pose.pose.position.x = self.goal_point[0]
        final_pose.pose.position.y = self.goal_point[1]
        if direction == "n":
            final_pose.pose.position.y = self.goal_point[1] + radius
            angle_to_goal = math.atan2(-1, 0)
        elif direction == "s":
            final_pose.pose.position.y = self.goal_point[1] - radius
            angle_to_goal = math.atan2(1, 0)
        elif direction == "w":
            final_pose.pose.position.x = self.goal_point[1] - radius
            angle_to_goal = math.atan2(0, 1)
        elif direction == "e":
            final_pose.pose.position.y = self.goal_point[1] + radius
            angle_to_goal = math.atan2(0, -1)

        final_pose.pose.orientation.z = math.sin(angle_to_goal / 2.0)
        final_pose.pose.orientation.w = math.cos(angle_to_goal / 2.0)
        path_msg.poses.append(final_pose)

        path_msg = self.polish_path(path_msg)
        self.path_publisher.publish(path_msg)

    def polish_path(self, path: Path) -> Path:
        print("TODO")

    def backup_planning(self, goal_position, distance_to_object):
        print("TODO")
        distance = math.sqrt(
            (self.start_point[0] - goal_position[0]) ** 2
            + (self.start_point[1] - goal_position[1]) ** 2
        )
        factor_dist_to_obj = 1 - distance_to_object / distance
        self.goal = (
            self.start_point[0] + (goal_position[0] - self.start_point[0]) * factor_dist_to_obj,
            self.start_point[0] + (goal_position[1] - self.start_point[1]) * factor_dist_to_obj,
        )
        self.backup_publish(self.goal)

    def backup_publish(self, point):
        goal_transform = TransformStamped()
        goal_transform.header.frame_id = "map"
        goal_transform.child_frame_id = "goal_position"
        goal_transform.header.stamp = self.get_clock().now().to_msg()

        # assign random point to transform
        goal_transform.transform.translation.x = point[0]
        goal_transform.transform.translation.y = point[1]

        self.backup_publisher.publish(goal_transform)
        self.get_logger().info(f"Published goal position: {point}")


def main():
    rclpy.init()
    node = PathPlanningNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
