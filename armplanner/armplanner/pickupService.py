#!/usr/bin/env python

import math
import cv2
from cv_bridge import CvBridge
import os

import numpy as np
from numpy import pi, cos, sin
import time

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from std_msgs.msg import Int64MultiArray, Int16MultiArray,MultiArrayDimension, MultiArrayLayout, String
#from armCameraObj import findObjectCenter
from sklearn.cluster import DBSCAN
from my_custom_interfaces.srv import Pickup, GetPosition  
from armplanner.kinematics3 import inverse_kinematics, compute_fk, translate_to_servo

class PickupService(Node):
    def __init__(self):
        super().__init__('robot_arm_node')

        self.servos_publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)
        
        self.detection_client = self.create_client(GetPosition, '/get_object_position')

        self.pickup_srv = self.create_service(Pickup, 'pickup', self.pickup_callback)

        self.backup = PointStamped()
        self.backup.point.x = 0.2
        self.backup.point.y = 0.0
        self.backup.point.z = -0.13
        self.plush_orientation = 'DEFAULT'

        # Possibly combine with the detection service:
        self.bridge = CvBridge()
        self.latest_frame = None
        self.subscription = self.create_subscription(
            Image, '/arm_camera/image_raw', self.image_callback, 10)
        
        # Loading calibration data
        curFolder = os.path.dirname(os.path.abspath(__file__))
        paramPath = os.path.join(curFolder, 'calibration2.npz')
        data = np.load(paramPath, allow_pickle=True)
        self.camMatrix = data['camMatrix']
        self.distCoeff = data['distCoeff']
        
        self.switch = 0 # Swap between blind and camera assisted pick for plushies
        
        # Confirm service is ready
        self.get_logger().info('Pickup service ready.')

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def pickup_callback(self, request, response):
        self.get_logger().info(f'Received pickup request for: {request.object_type}, {request.color}')
        if request.object_type == 'Cube' or request.object_type == 'Sphere':
            pass
        elif request.object_type == 'Plushie':
            self.backup.point.z = -0.09
        # Dummy position, ideally use detection logic
        obj_pos = PointStamped()
        obj_pos.point.x = 0.2
        obj_pos.point.y = 0.0
        obj_pos.point.z = -0.13

        # Begin detection process
        """ angles = translate_to_servo([0,70,65,95]) # arm camera angles
        angles = [int(angle1 * 100) for angle1 in angles]
        servos_angles_times1 = [[3000,12000,12000,12000,12000,12000, 2000,2000,2000,2000,2000,2000],
                            [3000,12000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]]
    
        msg1 = Int16MultiArray()
        msg1.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
    
        for angles in servos_angles_times1:
            self.get_logger().info(f'Angles: {angles}')
            msg1.data = angles
            self.servos_publisher.publish(msg1)
            self.get_logger().info(f'Published message: {msg1.data}')
            time.sleep(3) """
        """ if request.object_type == 'Cube' or request.object_type == 'Sphere':
            position = self.arm_detection3(request.object_type)
        elif request.object_type == 'Plushie':
            position = self.backup """
        position = self.arm_detection3(request.object_type)
        #print(position)
        if position == None:
            self.get_logger().error("Object detection failed!")
            # go to default position
            position = self.backup
            response.success = False
            response.message = "Object detection failed."
            return response
        elif position.point.x < 0.11:
            self.get_logger().error("Object detection failed! Too close to the robot")
            response.success = False
            response.message = "Object detection failed."
            position = self.backup
            #return response
        
        # Run IK and send servo commands
        angles, servo_angles = inverse_kinematics(position)
        print("End-effector position:", compute_fk(angles))
        self.control_servos(servo_angles, 'PICK', request.object_type)

        response.success = True
        response.message = f"Pickup done for: {request.object_type}, {request.color}"
        return response
    
    
        
    def createCombinedMask(self, s_step, v_step, hue_min, hue_max, hsv_frame):
        masks = []
        for s_min in range(0, 256, s_step):
            for v_min in range(0, 256, v_step):
                lower = np.array([hue_min, s_min, v_min])
                upper = np.array([hue_max, min(s_min + s_step - 1, 255), min(v_min + v_step - 1, 255)])
                mask = cv2.inRange(hsv_frame, lower, upper)
                masks.append(mask)
        final_mask = np.zeros_like(masks[0])
        for m in masks:
            final_mask = cv2.bitwise_or(final_mask, m)
        mask = final_mask
        return mask
    

    

    def arm_detection3(self, object_type):
        # detection logic
        plush_orientation = 'DEFAULT'
        start = time.time()
        frame = self.latest_frame
        if frame is None:
                self.get_logger().warn("No frame received yet!")
                return
        print('frame received')
        img = frame.copy()
        height, width = img.shape[:2]
        camMatrixNew,roi, = cv2.getOptimalNewCameraMatrix(self.camMatrix, self.distCoeff,(width,height),1, (width,height))
        imgUndist = cv2.undistort(img,self.camMatrix, self.distCoeff, None, camMatrixNew)
        
        # Crop the undistorted image
        x, y, w, h = roi
        cropped_img = imgUndist[y:y+h-34, x:x+w]
        cv2.imwrite('/home/happy/img.png', cropped_img)

        if object_type == 'Cube' or object_type == 'Sphere':
            


            # Convert the image to grayscale
            gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(16, 16))
            gray_img = clahe.apply(gray_img)

            # Apply Canny edge detection
            kernel = np.ones((5, 5), np.uint8)
            edges = cv2.Canny(gray_img, threshold1=50, threshold2=150)
            inverted_edges = cv2.bitwise_not(edges)
            maskk = cv2.erode(inverted_edges, kernel, iterations=1)

            cv2.imwrite('/home/happy/edges.png', inverted_edges)


            # Optionally, visualize the edges
            cv2.imwrite('/home/happy/maskk.png', maskk)
            #end = time.time()
            #print('time', end-start)

            points = np.column_stack(np.where(maskk > 0))
            cx,cy = 0, 0 # maybe change this
            avg_point = 0
            mask2 = maskk.copy()
            if points.size > 0:
                # Apply DBSCAN clustering
                clustering = DBSCAN(eps=2, min_samples=10).fit(points)  # Adjust 'eps' based on pixel scale
                
                # Get labels and find the largest cluster
                labels, counts = np.unique(clustering.labels_, return_counts=True)

                # Ignore noise points (-1 label) and find the largest cluster
                valid_labels = labels[labels != -1]
                
                filtered_clusters = []

                for label in valid_labels:
                    cluster_points = points[clustering.labels_ == label]
                    #print(label)
                    ys, xs = cluster_points[:, 0], cluster_points[:, 1]
                    width = xs.max() - xs.min()
                    height = ys.max() - ys.min()
                    #width < 80 and height < 80 and 
                    # Example condition: must be roughly square and not too thin
                    if object_type == 'Sphere':
                        if 0.8 < (width / height) < 1.2 and len(cluster_points) > 1500 and len(cluster_points) < 4000:
                            filtered_clusters.append((label, len(cluster_points)))
                    elif object_type == 'Cube':
                        if 0.8 < (width / height) < 1.2 and len(cluster_points) > 1000 and len(cluster_points) < 2500:
                            filtered_clusters.append((label, len(cluster_points)))

                if filtered_clusters:
                    # Choose largest valid cluster from filtered
                    best_label = max(filtered_clusters, key=lambda x: x[1])[0]
                    largest_cluster_points = points[clustering.labels_ == best_label]
                    cluster_size = len(largest_cluster_points)
                    print('cluster size',cluster_size)

                    # Compute centroid
                    avg_point = np.mean(largest_cluster_points, axis=0)
                    cx, cy = int(avg_point[1]), int(avg_point[0])

                    filtered_mask = np.zeros_like(mask2)
                    filtered_mask[largest_cluster_points[:, 0], largest_cluster_points[:, 1]] = 255

                else:
                    cx, cy = None, None
                    cluster_size = 0
                    filtered_mask = np.zeros_like(mask2)
            
            end = time.time()
            print('detection time:', end-start)
                
        
            
            

            #return msg

        if object_type == 'Plushie':

            start = time.time()
            img = cropped_img.copy()
            Z = img.reshape((-1,3))
            # convert to np.float32
            Z = np.float32(Z)
            # define criteria, number of clusters(K) and apply kmeans()
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            K = 3
            ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
            # Now convert back into uint8, and make original image
            center = np.uint8(center)
            res = center[label.flatten()]
            res2 = res.reshape((img.shape))
            end = time.time()
            print('kmeans time:', end-start)
            
            print('Plushie')
            # Implement Plushie detection logic here
            gray_img = cv2.cvtColor(res2, cv2.COLOR_BGR2GRAY)
            #_, binary = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            
            #gray_img = cv2.erode(binary, np.ones((5, 5), np.uint8), iterations=1)
            cv2.imwrite('/home/happy/res2.png', gray_img)

            # Apply Canny edge detection
            kernel = np.ones((5, 5), np.uint8)
            edges = cv2.Canny(gray_img, threshold1=50, threshold2=150)
            inverted_edges = cv2.bitwise_not(edges)
            maskk = cv2.erode(inverted_edges, kernel, iterations=1)

            # Optionally, visualize the edges
            cv2.imwrite('/home/happy/edges.png', inverted_edges)

            cv2.imwrite('/home/happy/maskk.png', maskk)

            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(maskk, connectivity=8)
            print('num_labels', num_labels, 'labels',labels, 'stats', stats, 'centroids', centroids)

            
            # Assuming maskk is your binary mask image
            max_solidity = 0
            contours, _ = cv2.findContours(maskk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Create a copy of the mask to draw contours on
            contours, _ = cv2.findContours(maskk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            high_solidity_mask = np.zeros_like(maskk)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 100:  # Skip tiny areas
                    continue
                    
                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                
                if hull_area == 0:
                    continue
                    
                solidity = float(area) / hull_area
                print(f"scontour solidity: {solidity}, area: {area}")
                
                if solidity > 0.6:  # High solidity threshold
                    cv2.drawContours(high_solidity_mask, [cnt], -1, 255, thickness=cv2.FILLED)
                    if solidity > max_solidity:
                        max_solidity = solidity

            maskk = high_solidity_mask
            cv2.imwrite('/home/happy/highmask.png', high_solidity_mask)
            points = np.column_stack(np.where(maskk > 0))
            cx,cy = 0, 0 # maybe change this
            avg_point = 0
            mask2 = maskk.copy()
            if points.size > 0:
                # Apply DBSCAN clustering
                clustering = DBSCAN(eps=2, min_samples=10).fit(points)  # Adjust 'eps' based on pixel scale
                
                # Get labels and find the largest cluster
                labels, counts = np.unique(clustering.labels_, return_counts=True)

                # Ignore noise points (-1 label) and find the largest cluster
                valid_labels = labels[labels != -1]
                
                filtered_clusters = []
                iteration =  0
                plush_orientation = {}
                for label in valid_labels:
                    cluster_points = points[clustering.labels_ == label]

                    ys, xs = cluster_points[:, 0], cluster_points[:, 1]
                    width = xs.max() - xs.min()
                    height = ys.max() - ys.min()
                    print(len(cluster_points), 'width', width, 'height', height)
                    edge_threshold = 20  # max allowed number of edge-near points
                    edge_margin = 5      # pixels considered "near the edge"

                    # Count how many cluster points are within the edge margin
                    near_top    = (cluster_points[:, 0] < edge_margin)
                    near_bottom = (cluster_points[:, 0] > maskk.shape[0] - edge_margin)
                    near_left   = (cluster_points[:, 1] < edge_margin)
                    near_right  = (cluster_points[:, 1] > maskk.shape[1] - edge_margin)

                    edge_point_count = np.sum(near_top | near_bottom | near_left | near_right)
                    #print('density', density, 'width/height', width/height, len(cluster_points))
                    #width < 80 and height < 80 and 
                    # Example condition: must be roughly square and not too thin
                    iteration += 1
                    if 0.65 < (width / height) < 1.5 and len(cluster_points) > 1000 and len(cluster_points) < 8000 and edge_point_count < edge_threshold:   
                        filtered_clusters.append((label, len(cluster_points)))
                    if 0.7 < (width / height) < 1.5 and len(cluster_points) > 500:
                        plush_orientation[label] = 'DEFAULT'
                    else:
                        plush_orientation[label] = 'DEFAULT'

                if filtered_clusters:
                    # Choose largest valid cluster from filtered
                    best_label = max(filtered_clusters, key=lambda x: x[1])[0]
                    largest_cluster_points = points[clustering.labels_ == best_label]
                    cluster_size = len(largest_cluster_points)
                    self.plush_orientation = plush_orientation[best_label]
                    print('cluster size',cluster_size)

                    # Compute centroid
                    avg_point = np.mean(largest_cluster_points, axis=0)
                    cx, cy = int(avg_point[1]), int(avg_point[0])

                    filtered_mask = np.zeros_like(mask2)
                    filtered_mask[largest_cluster_points[:, 0], largest_cluster_points[:, 1]] = 255

                else:
                    cx, cy = None, None
                    cluster_size = 0
                    filtered_mask = np.zeros_like(mask2)
            pass

        if cx == None or cy == None:
                print('No object detected')
                cx, cy = 0, 0
                return None
        # Compute the real-world coordinates of the centroid
        h, w = cropped_img.shape[:2]
        xnew = h - cy 
        ynew = w/2 - cx

        pos_x = xnew/h * 17.5/100 + 10/100  # KANSKE BORDE VARA TYP 17.6-17.7 ish
        pos_y = ynew/(w/2) * 25/200 
        if object_type == 'Cube' or object_type == 'Sphere':
            pos_z = -0.13
            pos_x = pos_x - 0.01
            if pos_y < 0:
                pos_y = pos_y + 1/100
            elif pos_y > 0:
                pos_y = pos_y - 1/100
        elif object_type == 'Plushie':
            pos_x = pos_x - 2/100
            if pos_y < 0:
                pos_y = pos_y + 1/100
            elif pos_y > 0:
                pos_y = pos_y - 1/100
            pos_z = -0.09


        # Return the position of the object
        msg = PointStamped()
        msg.point.x = float(pos_x)
        msg.point.y = float(pos_y)
        msg.point.z = pos_z # placeholder value
        self.get_logger().info(f"Object detected at: {msg.point.x}, {msg.point.y}, {msg.point.z}")


        # Draw a circle at the center
        cv2.circle(cropped_img, (cx, cy), 5, (0, 255, 0), -1)
        cv2.circle(cropped_img, (cy, cx), 5, (0, 0, 255), -1)  # Red dot for the centroid
        cv2.putText(cropped_img, f"({cx}, {cy})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) 

        cv2.circle(cropped_img,(int(w/2), int(h/2)), 5, (0, 255, 0), -1)
        cv2.putText(cropped_img, f"({w/2}, {h/2})", (int(w/2 + 10),int(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the processed frame
        original_image = cropped_img.copy()
        if len(original_image.shape) == 2:  # If grayscale, convert to BGR
            original_image = cv2.cvtColor(original_image, cv2.COLOR_GRAY2BGR)

        # Create a copy of the original image
        overlay = original_image.copy()

        # Highlight the largest cluster in red
        overlay[filtered_mask > 0] = [0, 255, 0]  # Set only cluster points to red

        # Blend the overlay with the original image
        alpha = 0.5  # Transparency factor
        result = cv2.addWeighted(original_image, 1 - alpha, overlay, alpha, 0)

        # Show the result
        print('showing result')
        cv2.imwrite("/home/happy/cluster_overlay.png", result)

        return msg


    def control_servos(self, angles, command, object_type):
        """ if command != 'PICK':
            return """

        valid_angles = [True, True, True, True]
        if angles[2] < 30:
            valid_angles[2] = False
        if angles[1] < 0:
            valid_angles[1] = False
        if angles[0] > 150:
            valid_angles[0] = False
        if angles[3] < 0:
            valid_angles[3] = False
        angles = [int(angle * 100) for angle in angles]
        # 12000
        if all (valid_angles):
            if self.plush_orientation != 'SIDE' and self.switch == 0 and object_type != 'Plushie':
                self.get_logger().info(f'Angles: {angles}')
                upright_open = [3000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]
                grab_pos = [3000,12000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]
                grab = [11000,12000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]
                upright_hold = [11000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]
                sequence = [upright_open, grab_pos, grab, upright_hold]
            elif self.plush_orientation != 'SIDE' and self.switch == 0:
                self.get_logger().info(f'Angles: {angles}')
                upright_open = [3000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]
                grab_pos = [3000,16500,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]
                grab = [11000,16500,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]
                upright_hold = [11000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]
                sequence = [upright_open, grab_pos, grab, upright_hold]
            elif self.plush_orientation == 'DEFAULT' and self.switch == 1:
                servo_6_pick = 13200
                sequence = [[3000, 12000, 12000, 12000, 12000, 12000, 2000, 2000, 2000, 2000, 2000, 2000], # Reset arm
                                   [3000, 12000, 12000, 12000, 12000, servo_6_pick, 2000, 2000, 2000, 2000, 2000, 2000], # Rotate base
                                   [3000, 16500, 3500, 12800, 4000, servo_6_pick, 2000, 2000, 2000, 2000, 2000, 2000], # Go down
                                   [12500, 16500, 3500, 12800, 4000, servo_6_pick, 2000, 2000, 2000, 2000, 2000, 2000], # Close gripper
                                   [12500, 12000, 12000, 12000, 12000, servo_6_pick, 2000, 2000, 2000, 2000, 2000, 2000]] # Go up
            elif self.plush_orientation == 'SIDE':
                self.get_logger().info(f'Angles: {angles}')
                print('SIDE')
                upright_open = [3000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]         
                grab_pos = [3000,3000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]
                grab = [11000,3000,angles[3],angles[2],angles[1],angles[0], 2000,2000,2000,2000,2000,2000]
                upright_hold = [11000,12000,12000,12000,12000, 12000, 2000,2000,2000,2000,2000,2000]
                sequence = [upright_open, grab_pos, grab, upright_hold]

            msg = Int16MultiArray()
            msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)

            for step in sequence:
                msg.data = step
                self.servos_publisher.publish(msg)
                time.sleep(3)

        #self.feedback_publisher.publish(String(data='SUCCESS'))'


    

def main(args=None):
    rclpy.init(args=args)
    node = PickupService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()