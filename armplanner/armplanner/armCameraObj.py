#!/usr/bin/env python

import math
import cv2
from cv_bridge import CvBridge


import numpy as np
from numpy import pi, cos, sin

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64MultiArray, Int8MultiArray, String
from sensor_msgs.msg import Image

class armCameraNode(Node):
    def __init__(self):
        super().__init__('arm_camera_detection')
        self.publisher = self.create_publisher(PoseStamped, '/object_position',10)

        
        self.latest_frame = None
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=False)
        self.subscriber1 = self.create_subscription(String, '/arm_camera_controller', self.findObjectCenter,10)
        self.subscriber2 = self.create_subscription(Image, '/arm_camera/image_raw',  self.image_callback, 10)

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        """ cv2.imshow("ROS2 Video Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            rclpy.shutdown() """

    def findObjectCenter(self, msg):
        if self.latest_frame is None:
            self.get_logger().warn("No frame received yet!")
            return

        frame = self.latest_frame
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow('frame', frame)
        cv2.waitKey(10000)
        obj = ['Plush', 'Gray']

        if obj[0] == 'Cube' or obj[0] == 'Sphere':
            print(obj[0])
            if obj [1] == 'Green':
                lower_color = np.array([35, 100, 50])  # Lower bound of green in HSV
                upper_color = np.array([85, 255, 255]) # Upper bound of green in HSV

            if obj [1] == 'Red':
                lower_color = np.array([35, 100, 50])  # Lower bound of red in HSV
                upper_color = np.array([85, 255, 255]) # Upper bound of red in HSV

            

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create a mask for the green object
            mask = cv2.inRange(hsv, lower_color, upper_color)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            points = np.column_stack(np.where(mask > 0))
            cx,cy = 0, 0 # maybe change this
            avg_point = 0
            if points.size > 0:
                avg_point = np.mean(points, axis=0)
                cx = int(avg_point[1])
                cy = int(avg_point[0])
                #print(x,y)
                
            #cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            #print(len(contours))
            hfov = 120  # Horizontal FOV
            d = 0.2 # Distance of camera from ground
            h, w = frame.shape[:2]
            print(h,w)
            vfov = 2*np.arctan(h/w * np.tan(hfov/2))
            cx_t = cx - w/2
            cy_t = -cy + h/2
            realw = 2 * d * np.tan(hfov/2)
            realh = 2 * d * np.tan(vfov/2)
            pixelw = realw/w
            pixelh = realh/h

            xreal = cx_t * pixelw
            yreal = cy_t * pixelh
            
            L1 = 0 # Distance from arm base to arm camera X (default config)
            L2 = 140/1000 # Distance from arm base to arm camera Y (default config)
            C = 0  # Center of camera coordinate 
            x_cam, y_cam = xreal, yreal
            pos_x = x_cam - C 
            pos_y = y_cam - C + L2
            
            # Draw a circle at the center
            """ cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"({cx}, {cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) 
            cv2.circle(frame, (int(w/2), int(h/2)), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"({w/2}, {h/2})", (int(w/2)+10,int(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  """
            #cv2.circle(frame, (ctotal[0], ctotal[1]), 5, (0, 255, 0), -1)
            #cv2.putText(frame, f"({ctotal[0]}, {ctotal[1]})", (ctotal[0] + 10, ctotal[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) 
            print('yobo')
            msg = PoseStamped()
            msg.pose.position.x = float(pos_x)
            msg.pose.position.y = float(pos_y)
            msg.pose.position.z = 0.1 # placeholder value

            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 0.0
            print(msg.pose.position.x,msg.pose.position.y)
            self.publisher.publish(msg)
            # Show the processed frame
            #cv2.imshow("Mask", mask)  # Show mask for debugging
            #cv2.imshow("Frame", frame)
            #cv2.waitKey(0)
        elif obj[0] == 'Plush':
            img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(img, (5, 5), 0)

            edges = cv2.Canny(blurred, 200, 250)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            output_image = frame.copy()
            centroids = []
            for contour in contours:
                if cv2.contourArea(contour) > 20:  # Filter by area to remove small contours
                    # Approximate the contour to a polygon
                    epsilon = 0.02 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)


                    M = cv2.moments(contour)
        
                    # Calculate the centroid (center) of the contour
                    if M["m00"] != 0:  # Avoid division by zero
                        cx = int(M["m10"] / M["m00"])  # X coordinate of the centroid
                        cy = int(M["m01"] / M["m00"])  # Y coordinate of the centroid
                    else:
                        cx, cy = 0, 0  # If area is zero (empty contour), set to (0, 0)
                    centroids.append([cx,cy])
                    # Draw the approximated polygon on the image
                    output_image = cv2.drawContours(output_image, [approx], -1, (0, 255, 0), 2)

            cx, cy = 0, 0
            for centroid in centroids:
                cx += centroid[0]
                cy += centroid[1]
            cx, cy = cx/len(centroids), cy/len(centroids)
            cx, cy = int(cx), int(cy)
            cv2.circle(output_image, (cx, cy), 5, (0, 0, 255), -1)  # Red dot for the centroid
            cv2.imshow('Edges', output_image)
            cv2.waitKey(10000)
            cv2.destroyAllWindows()
            print('hej')

        #return x, y

    def centerRobot(self,x,y,frame):
        h,w = frame.shape[:2]
        centerx = w/2
        centery = h/2
        while True:
            deltatheta = -np.arctan2((y-centery),(x-centerx))
            if deltatheta < 0:
                print('right')
                #turn right
            elif deltatheta > 0:
                print('left')
                #turn left

        


def main():
    rclpy.init()
    node = armCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
