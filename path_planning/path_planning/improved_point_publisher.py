#!/usr/bin/env python

import math

import rclpy
from rclpy.node import Node

import rclpy.time
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros

from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path

import tf2_geometry_msgs


class PointPublisherNode(Node):
    """
    Publishes the goal point for the controller to move towards.



    Args:
        Node (_type_): _description_
    """

    def __init__(self):
        super().__init__("point_publisher")  # Call the superclass constructor
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.publisher = self.create_publisher(TransformStamped, "/path/nextpos", 10)
        self.finish_publisher = self.create_publisher(
            TransformStamped, "/path/goal_reached", 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.backup_listener = self.create_subscription(
            TransformStamped, "/path/goal", self.backup_gtg, 10
        )

        self.path_listener = self.create_subscription(
            Path, "/path/planned_path", self.new_path, 10
        )

        self.position_reached = True
        self.goal_position_bu = TransformStamped()
        self.goals = []
        self.goal_position = TransformStamped()

        self.use_backup = True

        self.create_timer(0.5, self.go_to_point)

        # self.client = self.create_client(RandomPoint, "get_random_ws_point")
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Waiting for service to be available...")
        # self.request = RandomPoint.Request()

    def go_to_point(self):
        """
        publish transform to topic
        """
        if self.position_reached:
            return

        if self.use_backup:
            self.go_to_point_bu()
            return

        self.do_broadcast()
        rclpy.spin_once(self)

        # init
        goal_transform = self.goal_position
        time = self.goal_position.header.stamp
        robot_frame = "base_link"
        goal_frame = goal_transform.child_frame_id
        goal_margin_translational = 0.05
        goal_margin_rotational = math.pi / 10

        # Wait for the transform asynchronously
        compared_transform = self.buffer.wait_for_transform_async(
            target_frame=robot_frame, source_frame=goal_frame, time=time
        )

        rclpy.spin_until_future_complete(self, compared_transform, timeout_sec=1)

        # Check if the future completed successfully
        if not (compared_transform.done()):  # and compared_transform.result()
            self.get_logger().error(
                f"Transform future did not complete successfully for time {time}"
            )
            return

        try:
            # transform translation and rotation
            finished_transform = compared_transform.result()
            comp_translation = finished_transform.transform.translation
            comp_rotation = euler_from_quaternion(finished_transform.transform.rotation)
            distance_to_point = math.sqrt(comp_translation.x**2 + comp_translation.y**2)

            if distance_to_point < goal_margin_translational:
                if not self.goals:
                    if abs(comp_rotation[2]) < goal_margin_rotational:
                        self.position_reached = True
                        self.get_logger().info(
                            f"Position {[goal_transform.transform.translation.x, goal_transform.transform.translation.y, goal_transform.transform.rotation.z]} has been reached!"
                        )
                        self.finish_publisher.publish(self.goal_position)
                    else:
                        self.publisher.publish(goal_transform)
                else:
                    goal_transform = self.pop_goals()
                    self.publisher.publish(goal_transform)

            else:
                self.publisher.publish(goal_transform)

            return
        except Exception as ex:
            # Log any errors (this will only log broadcasting issues now)
            self.get_logger().error(f"Error: {ex}")
            return

    def go_to_point_bu(self):

        self.do_broadcast()
        rclpy.spin_once(self)

        # init
        goal_transform = self.goal_position_bu
        time = self.goal_position_bu.header.stamp
        robot_frame = "base_link"
        goal_frame = goal_transform.child_frame_id
        goal_margin_translational = 0.05

        # Wait for the transform asynchronously
        compared_transform = self.buffer.wait_for_transform_async(
            target_frame=robot_frame, source_frame=goal_frame, time=time
        )

        rclpy.spin_until_future_complete(self, compared_transform, timeout_sec=2)

        # Check if the future completed successfully
        if not (compared_transform.done()):  # and compared_transform.result()
            self.get_logger().error(
                f"Transform future did not complete successfully for time {time}"
            )
            return

        try:
            finished_transform = compared_transform.result()
            comp_translation = finished_transform.transform.translation
            distance_to_point = math.sqrt(comp_translation.x**2 + comp_translation.y**2)

            if distance_to_point < goal_margin_translational * 2:
                self.position_reached = True
                self.get_logger().info(
                    f"Position {[goal_transform.transform.translation.x, goal_transform.transform.translation.y, goal_transform.transform.rotation.z]} has been reached!"
                )
                self.finish_publisher.publish(self.goal_position_bu)
            else:
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

        ws_point = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, ws_point)

        random_x = ws_point.result().x
        random_y = ws_point.result().y
        random_rot = ws_point.result().theta
        print(random_x, random_y, random_rot)

        # initialize new transform
        goal_transform = TransformStamped()
        goal_transform.header.frame_id = "map"
        goal_transform.child_frame_id = "goal_position"
        goal_transform.header.stamp = self.get_clock().now().to_msg()

        # assign random point to transform
        goal_transform.transform.translation.x = random_x
        goal_transform.transform.translation.y = random_y
        random_quaternion = quaternion_from_euler(0, 0, random_rot)
        goal_transform.transform.rotation.z = random_quaternion[2]
        goal_transform.transform.rotation.w = random_quaternion[3]

        self.goal_position_bu = goal_transform

        print(
            f"GOT NEW POINT:\n{[goal_transform.transform.translation.x, goal_transform.transform.translation.y, goal_transform.transform.rotation.z]}"
        )
        self.get_logger().info(
            f"GOT NEW POINT(self):\n{[self.goal_position_bu.transform.translation.x, self.goal_position_bu.transform.translation.y, self.goal_position_bu.transform.rotation.z]}"
        )
        return self.goal_position_bu

    def pop_goals(self) -> TransformStamped:
        next_pose = self.goals.pop(0)
        next_goal = TransformStamped()
        next_goal.transform.translation = next_pose.pose.position
        next_goal.transform.rotation = next_pose.pose.orientation
        next_goal.header.stamp = self.get_clock().now().to_msg()
        next_goal.header.frame_id = "map"
        next_goal.child_frame_id = "goal_position"
        next_goal = self.goal_position

        return next_goal

    def backup_gtg(self, msg: TransformStamped):
        print("TODO")
        self.position_reached = False
        goal_transform = msg
        self.goal_position_bu = goal_transform

        print(
            f"GOT NEW POINT:\n{[goal_transform.transform.translation.x, goal_transform.transform.translation.y, goal_transform.transform.rotation.z]}"
        )
        self.get_logger().info(
            f"GOT NEW POINT(self):\n{[self.goal_position_bu.transform.translation.x, self.goal_position_bu.transform.translation.y, self.goal_position_bu.transform.rotation.z]}"
        )
        self.do_broadcast()
        return self.goal_position_bu

    def new_path(self, msg: Path):
        poses = msg.poses
        self.goals = poses
        self.position_reached = False
        self.pop_goals()

    def do_broadcast(self):
        if self.use_backup:
            goal_transform = self.goal_position_bu
            self.goal_position_bu.header.stamp = self.get_clock().now().to_msg()
            goal_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(goal_transform)
        else:
            goal_transform = self.goal_position
            self.goal_position.header.stamp = self.get_clock().now().to_msg()
            goal_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(goal_transform)


def main():
    rclpy.init()
    node = PointPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
