#!/usr/bin/env python

import math

import os
import csv

import rclpy
from rclpy.node import Node

import rclpy.time
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros

from geometry_msgs.msg import TransformStamped, Pose
from visualization_msgs.msg import Marker

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool

from enum import Enum

from builtin_interfaces.msg import Duration


class States(Enum):
    INIT = 1
    MTPU = 2
    PU = 3
    MTBOX = 4
    PLACE = 5


class Collection(Node):
    def __init__(self):
        super().__init__("automaton")
        self.state = States.INIT
        self.arm_subscription = self.create_subscription(
            Bool, "/arm/arm_done", self.arm_callback, 10
        )
        self.move_subscription = self.create_subscription(
            TransformStamped, "/path/goal_reached", self.move_callback, 10
        )

        self.goalPosition = Pose()
        self.obj_index = 0

        package_share_directory = get_package_share_directory("path_planning")
        self.ws_file = os.path.join(package_share_directory, "resource", "map_1.tsv")
        self.objects, self.boxes = self.read_tsv(self.ws_file)

        self.pu_publisher = self.create_publisher(
            TransformStamped, "/arm/obj_pos", 10
        )  # self.create_publisher(Bool, '/arm/pickup', 10)
        self.place_publisher = self.create_publisher(Bool, "/arm/place", 10)
        self.object_publisher = self.create_publisher(Pose, "new_object_pos", 10)
        self.box_publisher = self.create_publisher(Pose, "new_box_pos", 10)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)

        self.marker_publisher = self.create_publisher(
            Marker, "/object_marker", 10
        )

        self.do_init()

    def arm_callback(self, msg: Bool):
        if msg:
            if self.state == States.PU:
                self.state = States.MTBOX
                if self.boxes:
                    self.goalPosition = self.get_new_box()
                    self.box_publisher.publish(self.goalPosition)
                    print(
                        f"Next BOX: {[self.goalPosition.x, self.goalPosition.y, self.goalPosition.theta]}"
                    )
                else:
                    self.get_logger().info(
                        "NO BOXES WERE FOUND. ROBOT RETURNS TO ORIGIN."
                    )
                    self.goalPosition.x = 0
                    self.goalPosition.y = 0
                    self.goalPosition.theta = 0
                    self.box_publisher.publish(self.goalPosition)

            elif self.state == States.PLACE:
                self.state = States.INIT
                self.do_init()

    def move_callback(self, msg: TransformStamped):
        # if self.goalPosition.transform == msg.transform:
        if self.state == States.MTPU:
            self.state = States.PU
            self.pu_publisher.publish(self.goalPosition)
            self.objects.remove(self.obj_index)

        elif self.state == States.MTBOX:
            self.state = States.PLACE
            self.place_publisher.publish(True)

    def do_init(self):
        if not self.boxes:
            self.get_logger().error("NO BOXES WERE FOUND")
        if self.objects:
            self.state = States.MTPU
            self.goalPosition = self.get_new_object()
            self.object_publisher.publish(self.goalPosition)
            print(
                f"Next Object: {[self.goalPosition.x, self.goalPosition.y, self.goalPosition.theta]}"
            )
        else:
            self.get_logger().info("NO OBJECTS WERE FOUND. ROBOT RETURNS TO ORIGIN.")
            self.goalPosition.x = 0
            self.goalPosition.y = 0
            self.goalPosition.theta = 0
            self.object_publisher.publish(self.goalPosition)

    def get_new_object(self):
        current_x, current_y = self.get_current_position()

        shortest_dist = 3000
        obj_index = 0
        for index, object in enumerate(self.objects):
            obj_x, obj_y = object[0], object[1]
            distance = math.sqrt((current_x - obj_x) ** 2 + (current_y - obj_y) ** 2)
            if distance < shortest_dist:
                shortest_dist = distance
                obj_index = index

        self.obj_index = obj_index
        goal_point = Pose()
        position = self.objects[obj_index]
        goal_point.position.x = position[0]
        goal_point.position.y = position[1]
        goal_point.orientation = quaternion_from_euler(0, 0, position[2])
        return goal_point

    def get_new_box(self):
        current_x, current_y = self.get_current_position()

        shortest_dist = 3000
        box_index = 0
        for index, box in enumerate(self.boxes):
            box_x, box_y = box[0], box[1]
            distance = math.sqrt((current_x - box_x) ** 2 + (current_y - box_y) ** 2)
            if distance < shortest_dist:
                shortest_dist = distance
                box_index = index
        goal_point = Pose()
        position = self.objects[obj_index]
        goal_point.position.x = position[0]
        goal_point.position.y = position[1]
        goal_point.orientation = quaternion_from_euler(0, 0, position[2])
        return goal_point

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
        self.visualize_things()
        return objects, boxes

    def visualize_things(self):
        marker_array = []
        # Visualize objects
        for i, obj in enumerate(self.objects):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.lifetime = Duration(seconds=0) 
            marker.pose.position.x = obj[0]
            marker.pose.position.y = obj[1]
            marker.pose.position.z = 0.5  # Adjust height if needed
            marker.pose.orientation = quaternion_from_euler(0, 0, obj[2])
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.append(marker)
        
        # Visualize boxes
        for i, box in enumerate(self.boxes):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "boxes"
            marker.id = i + len(self.objects)  # Ensure unique IDs
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.lifetime = Duration(seconds=0) 
            marker.pose.position.x = box[0]
            marker.pose.position.y = box[1]
            marker.pose.position.z = 0.5  # Adjust height if needed
            marker.pose.orientation = quaternion_from_euler(0, 0, box[2])
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.append(marker)

        for marker in marker_array:
            self.marker_publisher.publish(marker)

def main():
    rclpy.init()
    node = Collection()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
