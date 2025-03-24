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

from robp_interfaces.msg import DutyCycles

import tf2_geometry_msgs
_path.duty_cycle_left}, {motor_msg.duty_cycle_right}")
            # print(self._pub.topic)
            return

        except Exception as ex:
            # Log any errors (this will only log broadcasting issues now)
            self.get_logger().error(
                f"Failed to move towards position: {msg.transform} \n {ex}"
            )
            return

    def nextpos_callback(self, msg: TransformStamped):
        self.goal_position = msg
        # init
        time = msg.header.stamp
        robot_frame = "base_link"
        goal_frame = "map"
        v = 0
        w = 0

        if self.goal_position.transform.translation.x != self.x_g:
            self.x_g = self.goal_position.transform.translation.x
            self.y_g = self.goal_position.transform.translation.y
            self.reset_rotation = True
            self.phase_one = 1

        alpha, beta, theta_final = euler_from_quaternion(
            [
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w,
            ]
        )
        compared_transform = self.buffer.wait_for_transform_async(
            target_frame=goal_frame, source_frame=robot_frame, time=time
        )
        rclpy.spin_until_future_complete(self, compared_transform, timeout_sec=0.5)

        # Check if the future completed successfully
        if not compared_transform.done():
            self.get_logger().error(
                f"Transform future did not complete successfully between {robot_frame} and {goal_frame}."
            )
            return
        else:
            finished_transform = compared_transform.result()
            current_translation = finished_transform.transform.translation
            current_rotation = finished_transform.transform.rotation
            x = current_translation.x
            y = current_translation.y
            alpha, beta, theta = euler_from_quaternion(
                [
                    current_rotation.x,
                    current_rotation.y,
                    current_rotation.z,
                    current_rotation.w,
                ]
            )

            distance_to_point = math.sqrt((x - self.x_g) ** 2 + (y - self.y_g) ** 2)
            delta_x = self.x_g - x
            delta_y = self.y_g - y
            if self.reset_rotation:
                self.theta_g = math.atan2(delta_y, delta_x)
                self.x_0 = x
                self.y_0 = y
                self.reset_rotation = False

            delta_theta = self.theta_g - theta
            print(delta_theta)

        try:
            if self.phase_one == 1:
                if (abs(theta - self.theta_g) > self.goal_margin_rotational):
                    w = self.p_rotation * delta_theta
                    v = 0
                    # v = self.p_trans_rotation * (cos(theta_))
                    print(f"Rotating towards {[delta_x, delta_y]}")
                    print(f"Rotating w: {w}")
                else:
                    self.phase_one = 2
                    self.get_logger().info("Positioned towards goal")
            elif self.phase_one == 2:
                if distance_to_point > self.goal_margin_translational:
                    v = self.p_translation * (
                        delta_x * math.cos(self.theta_g)
                        + delta_y * math.sin(self.theta_g)
                    )
                    # w = self.p_rot_translation * (math.sin(delta_theta*))
                    dp = math.sin(self.theta_g) * (
                        x + self.p * math.cos(theta) - self.x_0
                    ) - math.cos(self.theta_g) * (
                        y + self.p * math.sin(theta) - self.y_0
                    )
                    w = self.p_rotation_two * dp
                    print(f"Moving towards {[delta_x, delta_y]}")
                    print(f"v: {v}, w: {w}")
                else:
                    self.phase_one = 3
                    self.get_logger().info(
                        "Position reached. Correct orientation to be assigned."
                    )
            elif self.phase_one == 3:
                if abs(theta_final - theta) > self.goal_margin_rotational:
                    delta_theta = theta_final - theta
                    w = self.p_rotation * delta_theta
                    v = 0
                    print(f"AT GOAL\nROTATING DIST: {theta_final}")
                else:
                    print("Already AT GOAL")
                    v, w = 0, 0
            print(f"v: {v}, w: {w}")

            # Damping
            v = v * self.v_damper
            w = w * self.w_damper

            if abs(v) > 1.5:
                v = np.sign(v) * 1.5
            if abs(w) > 1.5:
                w = np.sign(w) * 1.5
            # Convert to duty cycles
            motor_msg = DutyCycles()
            motor_msg.duty_cycle_left = (v - 0.5 * w) * self.cycle_damping
            motor_msg.duty_cycle_right = (v + 0.5 * w) * self.cycle_damping

            motor_msg.header.stamp = self.get_clock().now().to_msg()  # msg.header.stamp

            # Publish the message
            self._pub.publish(motor_msg)
            # self.get_logger().info(f"Published: {motor_msg.duty_cycle_left}, {motor_msg.duty_cycle_right}")
            # print(self._pub.topic)
            return

        except Exception as ex:
            # Log any errors (this will only log broadcasting issues now)
            self.get_logger().error(
                f"Failed to move towards position: {msg.transform} \n {ex}"
            )
            return


def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
