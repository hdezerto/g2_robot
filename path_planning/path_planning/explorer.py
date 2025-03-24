#!/usr/bin/env python

import math

import os
import csv

import random
import numpy as np

import rclpy
from rclpy.node import Node

import rclpy.time
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped, Pose

from ament_index_python.packages import get_package_share_directory

from enum import Enum

from shapely.geometry import Polygon, Point


class States(Enum):
    INIT = 1
    REQU = 2  # Request new point
    MOVE = 3  # Move to point
    DETE = 4  # Follow detection direction


class Explorer(Node):
    def __init__(self):
        super().__init__("automaton")

        # Random point service
        workspace_filename = "workspace_2.tsv"
        package_share_directory = get_package_share_directory("path_planning")
        self.ws_file = os.path.join(
            package_share_directory, "resource", workspace_filename
        )
        self.ws_vertices = self.read_tsv()
        self.ws_polygon = Polygon(self.ws_vertices)
        self.ws_polygon_buffered = self.ws_polygon.buffer(-20)

        # Initialize state
        self.state = States.INIT
        self.move_subscription = self.create_subscription(
            TransformStamped, "/path/goal_reached", self.move_callback, 10
        )

        self.goalPose = Pose()

        self.pose_publisher = self.create_publisher(Pose, "path/new_explore_pos", 10)

        self.create_subscription(
            Pose, "detection/something", self.detection_callback, 10
        )
        self.detected_object = Pose()
        self.detect_pose_publisher = self.create_publisher(
            Pose, "path/new_detection_pos", 10
        )

        # self.client = self.create_client(Pose, "get_random_ws_point")
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Waiting for service to be available...")
        # self.request = Pose.Request()

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)

        self.last_position = (0, 0)

        self.do_init()

    def do_init(self):
        self.state = States.REQU
        self.request_point()

    def request_point(self):
        ws_pose = self.get_new_point()

        random_x = ws_pose.position.x
        random_y = ws_pose.position.y
        self.goalPose = ws_pose

        self.state = States.MOVE
        self.do_publish_move()

    def do_publish_move(self):
        self.pose_publisher.publish(self.goalPose)
        self.get_logger().info(f"Published Position: {self.goalPose}")

    def move_callback(self, msg: TransformStamped):
        if self.state == States.MOVE:
            self.state = States.REQU
            self.request_point()
        elif self.state == States.DETE:
            self.state = States.MOVE
            self.do_publish_move()
        else:
            self.get_logger().warning("Unexpected callback")

    def detection_callback(self, msg: Pose):
        self.state = States.DETE

        dist = math.sqrt(
            (msg.position.x - self.detected_object.position.x) ** 2
            + (msg.position.y - self.detected_object.position.y) ** 2
        )
        if dist > 0.2:
            self.detected_object = msg
            self.detect_pose_publisher.publish(self.detected_object)
            self.get_logger().info(
                f"Published Detection Position: {self.detected_object}"
            )

    def get_current_position(self):
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
                f"Transform future did not complete successfully for time {time}.\nUsing latest transform"
            )
            time = rclpy.time.Time(seconds=0)
            current_position_future = self.buffer.wait_for_transform_async(
                target_frame="map", source_frame="base_link", time=time
            )
            rclpy.spin_until_future_complete(
                self, current_position_future, timeout_sec=2
            )
            if not (current_position_future.done()):
                self.get_logger().error(
                    f"Using latest transform time failed. Using (0,0)"
                )
                current_x, current_y = self.last_position

        else:
            current_position = current_position_future.result()
            current_x = current_position.transform.translation.x
            current_y = current_position.transform.translation.y
            self.last_position = (current_x, current_y)
        return current_x, current_y

    def get_new_point(self):
        """
        generate new point

        assign random point in workspace (within a margin) to self.goal_position.transform.translation.x/y
        #/ assign random orientation within [0, 2pi] as quaternion to goal_position.transform.rotation.z

        """
        pose = Pose()
        random_x, random_y = self.point_in_ws()
        random_rot = random.uniform(0, 2 * math.pi)
        pose.position.x = random_x
        pose.position.y = random_y
        quat = quaternion_from_euler(0, 0, random_rot)
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose

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
    node = Explorer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
