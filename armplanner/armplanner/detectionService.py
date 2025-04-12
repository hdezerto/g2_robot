#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String
from my_custom_interfaces.srv import GetPosition
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArmCameraNode(Node):
    def __init__(self):
        super().__init__('arm_camera_service_node')

        self.bridge = CvBridge()
        self.latest_frame = None

        self.subscription = self.create_subscription(
            Image, '/arm_camera/image_raw', self.image_callback, 10)

        self.service = self.create_service(
            GetPosition, '/get_object_position', self.handle_get_position)


        self.get_logger().info('Camera service ready.')

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def handle_get_position(self, request, response):
        object_type = request.object_type
        color = request.color
        if self.latest_frame is None:
            self.get_logger().warn("No frame received yet!")
            #return False
            msg1 = PointStamped()
            msg1.point.x = 0.0
            msg1.point.y = 0.0
            msg1.point.z = 0.0
            response.pose = msg1
            response.success = False
            response.message = "No frame received yet!"
            return response
        frame = self.latest_frame
        print('frame received')
        # Process the image to find the object
        if object_type == 'Cube' or object_type == 'Sphere':
            print(object_type)
            # Convert the image to HSV color space
            if color == 'Green':
                lower_color = np.array([35, 100, 50])  # Lower bound of green in HSV
                upper_color = np.array([85, 255, 255]) # Upper bound of green in HSV

            elif color == 'Red':
                lower_color = np.array([35, 100, 50])  # Lower bound of red in HSV
                upper_color = np.array([85, 255, 255]) # Upper bound of red in HSV

            elif color == 'Blue':
                lower_color = np.array([100, 150, 0])   # Lower bound of blue in HSV        
                upper_color = np.array([140, 255, 255]) # Upper bound of blue in HSV

            else:
                self.get_logger().warn("Unsupported color!")
                return False
            # Create a mask for the specified color
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_frame, lower_color, upper_color)

            # missing code to find the object center, clustering etc. add tomorrow



            # Return the position of the object
            msg = PointStamped()
            msg.point.x = float(pos_x)
            msg.point.y = float(pos_y)
            msg.point.z = -0.13 # placeholder value
            response.position.header.frame_id = "camera_frame"
            response.position.header.stamp = self.get_clock().now().to_msg()
            response.position.point = msg.point
            self.get_logger().info(f"Object detected at: {msg.point.x}, {msg.point.y}, {msg.point.z}")
        elif object_type == 'Plush':
            # Handle plush object detection
            # Implement the logic for plush object detection here
            self.get_logger().info("Plush object detected.")
            response.position.point.x = 0.0
            response.position.point.y = 0.0
            response.position.point.z = 0.0     

def main(args=None):
    rclpy.init(args=args)
    node = ArmCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()