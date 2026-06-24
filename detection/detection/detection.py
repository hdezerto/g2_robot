#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import ctypes
import struct
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import os
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from sklearn.decomposition import PCA

from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from sensor_msgs_py.point_cloud2 import create_cloud # Convert colors to a single float value representing RGB
import time

from detection_interfaces.msg import DetectionMsg




# ---------- TUNABLE PARAMETERS ----------

N_THRESHOLD = 8  # Process every N_THRESHOLD messages (to reduce processing load) TEST
MAX_DISTANCE = 0.9 # Maximum distance from the camera [m] TEST
MIN_DISTANCE = 0.04 # Minimum distance from the camera [m] TEST

MAX_HEIGHT = 0.087 # Maximum height from the floor [m] TEST
MIN_HEIGHT = -0.065 # Minimum height from the floor [m] TEST

# ----------------------------------------


class PointCloudDetection(Node):

    def __init__(self):
        super().__init__('PointCloudDetection_node')

        self.mat = None

        self.get_logger().info(f"Init PointCloudDetection node")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self, spin_thread=True)

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.sub2 = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            qos_profile
        )

        self.pub = self.create_publisher(PointCloud2, '/depth_points_filtered', 100)


        self.cluster_publisher = self.create_publisher(PointCloud2, '/clusters', 10)


        self.detection_publisher = self.create_publisher(DetectionMsg, '/detections', 10)  # Publisher for detections in exploration

        self.message_counter = 0
        self.detected_boxes = []



    def cloud_callback(self, msg: PointCloud2):

        # Increment the message counter
        self.message_counter += 1
        # Only process every N_THRESHOLD messages. Fequency of /camera/camera/depth/color/points is around 15 point clouds per second
        if self.message_counter % N_THRESHOLD != 0:
            return
        # Reset the counter to avoid overflow
        self.message_counter = 0

        start_time = time.time() # DEBUGGING EFFICIENCY

        # Read point cloud data from the message
        points_data = pc2.read_points_numpy(msg, skip_nans=True)

        # Extract XYZ coordinates from the point cloud
        points = points_data[:, :3]  # Shape (N, 3)
        
        # Compute Euclidean distance of each point from the origin
        distances = np.linalg.norm(points, axis=1)

        # Create a boolean mask to filter points:
        # - Points within max_dist from the sensor
        # - Points above the floor (0.01 < y < 0.085) (y-axis points downwards)
        mask = (distances > MIN_DISTANCE) & (distances < MAX_DISTANCE) & (points[:, 1] < MAX_HEIGHT) & (points[:, 1] > MIN_HEIGHT)  # TEST
        
        # Apply the mask to filter points before processing colors
        points = points[mask]



        #print(f"Y min: {np.min(points[:,1])}, Y max: {np.max(points[:,1])}") # TO TEST THE FLOOR RANGE

        # Extract color information from the point cloud
        # The color is stored as a floating-point number in the 4th column
        color_floats = points_data[mask, 3].view(np.uint32)  # Convert float to uint32 directly

        # Extract RGB channels using bitwise operations
        red = (color_floats >> 16) & 255
        green = (color_floats >> 8) & 255
        blue = color_floats & 255

        # Normalize colors to the range [0, 1] for consistency
        colors = np.stack((red, green, blue), axis=1).astype(np.float32) / 255  

        # Convert RGB values to packed 32-bit float format (used by ROS)
        # | 31-24 | 23-16 | 15-8 | 7-0  |
        # | Alpha |  Red  | Green | Blue |
        # The packed format stores colors as a single float value
        rgb_colors = (red << 16 | green << 8 | blue).astype(np.uint32).view(np.float32).reshape(-1, 1)
        
        # Combine filtered points with their corresponding colors
        points_with_colors = np.hstack((points, rgb_colors))  # Shape (N, 4)

        # Define the PointCloud2 message fields
        # Each point consists of (x, y, z, rgb) with FLOAT32 data type
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # Create a new PointCloud2 message with the filtered points and colors
        filtered_cloud_msg = create_cloud(msg.header, fields, points_with_colors)

        # Publish the filtered point cloud
        self.pub.publish(filtered_cloud_msg)

        # Perform spatial clustering using DBSCAN
        labels = self.dbscan(points, eps=0.05, min_samples=200) # CHECK
        
        # Publish clusters using the original header
        self.publish_clusters(points, labels, msg.header) # CAN BE COMMENTED OUT AFTER TESTING

        valid_labels = labels[labels != -1] # Filter out noise points (label = -1)
        unique_labels = np.unique(valid_labels)

        # Define HSV [Hue (0-179), Saturation(0-255), Value(0-255)] ranges for filtering color ranges for red, green, and blue
        lower_red, upper_red = np.array([2, 230, 95]), np.array([2, 240, 100])
        lower_green1, upper_green1 = np.array([81, 100, 44]), np.array([84, 255, 105])
        lower_green2, upper_green2 = np.array([73, 210, 90]), np.array([74, 240, 120])
        lower_blue, upper_blue = np.array([99, 254, 75]), np.array([99, 255, 80])

        # Iterate over each cluster
        for cluster_label in unique_labels:

            # Extract points and colors for the current cluster
            cluster_mask = (labels == cluster_label)
            cluster_points = points[cluster_mask]
            cluster_colors = colors[cluster_mask]

            # Convert RGB to HSV for the current cluster
            rgb_colors = cluster_colors * 255  # Scale back to 0-255
            # hsv_colors is a 2D array with shape (N, 3) where N is the number of points
            hsv_colors = cv2.cvtColor(rgb_colors.reshape(1, -1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV).reshape(-1, 3)

            # Create masks for the current cluster
            red_mask = ((hsv_colors[:, 0] >= lower_red[0]) & (hsv_colors[:, 0] <= upper_red[0]))
            green_mask = (hsv_colors[:, 0] >= lower_green1[0]) & (hsv_colors[:, 0] <= upper_green1[0]) | \
                        ((hsv_colors[:, 0] >= lower_green2[0]) & (hsv_colors[:, 0] <= upper_green2[0]))
            blue_mask = (hsv_colors[:, 0] >= lower_blue[0]) & (hsv_colors[:, 0] <= upper_blue[0])

            # Apply masks for the current cluster
            red_points = cluster_points[red_mask]
            green_points = cluster_points[green_mask]
            blue_points = cluster_points[blue_mask]

            # Calculate the total number of points in the cluster
            total_points = len(cluster_points)

            # Calculate the ratio of red, green, and blue points
            red_ratio = len(red_points) / total_points
            green_ratio = len(green_points) / total_points
            blue_ratio = len(blue_points) / total_points

            pure_red = pure_green = pure_blue  = False
            
            # Check if the cluster is predominantly red, green, blue or brown
            if red_ratio > 0.01 and green_ratio == 0.0 and blue_ratio == 0.0:
                pure_red = True
            elif green_ratio > 0.01 and red_ratio == 0.0 and blue_ratio == 0.0:
                pure_green = True
            elif blue_ratio > 0.01 and red_ratio == 0.0 and green_ratio == 0.0:
                pure_blue = True
            
            x, y, z = np.mean(cluster_points, axis=0) # Calculate the centroid of the cluster

            x_map, y_map, _ = self.transform_to_map(x, z, msg.header.stamp)

            # Disabled during collection because nearby boxes caused false object rejections.
            # if self.is_near_box(x_map, y_map):
            #     self.get_logger().info(f'Cluster is near a box!')
            #     continue

            # Classify based on floor contact points for the current cluster
            object_type = self.classify_based_on_floor_contact(cluster_points)

            if pure_red or pure_green or pure_blue:
                if object_type == "sphere":
                    self.get_logger().info(f'Cluster  is a sphere!')
                    self.create_object('sphere', x, z + 0.02, 0.0, msg.header.stamp)
                elif object_type == "cube":
                    self.get_logger().info(f' Cluster is a cube!')
                    self.create_object('cube', x, z + 0.02, 0.0, msg.header.stamp)
                elif object_type == "unknown":
                    self.get_logger().info(f'Object not identified :(!')


            elif self.is_box(cluster_points):  # If detected object is a box
                self.get_logger().info(f'📦 Cluster is a box!')

                # Compute the orientation angle of the box
                angle = self.estimate_box_orientation(cluster_points)
                
                # Store box with angle information
                if angle == 0.0:
                    self.create_object('box', x, z + 0.08, angle, msg.header.stamp)

                elif angle == 90.0:
                    self.create_object('box', x, z + 0.12, angle, msg.header.stamp)

                else:
                    self.create_object('box', x, z + 0.08, angle, msg.header.stamp)

            elif self.is_plushie(cluster_points): 
                self.get_logger().info(f'🧸 Cluster  is a plushie!')
                self.create_object('plushie', x, z + 0.01, 0.0, msg.header.stamp)

            else:
                self.get_logger().info(f'Cluster is NOT a recognized object.')


            # ------------ TIMER FOR EFFICIENCY CHECK (move where desired) ------------
            end_time = time.time()
            #self.get_logger().info(f"Processing time: {end_time - start_time:.4f} seconds")
            # ------------------------------------------------------------------------


    # v ------------------ FUNCTIONS BELOW ------------------ v
    def is_near_box(self, x, y, radius=0.30):
        for box_x, box_y  in self.detected_boxes:
            distance = np.sqrt((x - box_x) ** 2 + (y - box_y) ** 2)
            #self.get_logger().info(f"Distance to box at ({box_x}, {box_y}): {distance:.3f}")
            if distance <= radius:
                return True
        return False

    def is_plushie(self, cluster_points):

        MAX_SIZE = 0.09
        MIN_SIZE = 0.045
        TOLERANCE = 0.04  # 4 cm tolerance

        # Calculate HORIZONTAL DIMENSIONS (X-Y plane)
        xy_points = cluster_points[:, [0, 1]]  # This extracts x and y coordinates
        pca = PCA(n_components=2)
        pca.fit(xy_points)
        
        # Project points onto horizontal principal axes
        projected = xy_points @ pca.components_.T
        max_component_size = np.ptp(projected[:, 0])  # Primary horizontal dimension
        min_component_size = np.ptp(projected[:, 1])   # Secondary horizontal dimension
        
        # Match dimensions to expected length/width
        dim_match = (
            (abs(max_component_size - MAX_SIZE) < TOLERANCE) and
            (abs(min_component_size - MIN_SIZE) < TOLERANCE)
        )

        # Debug output
        # self.get_logger().info(
        #     f"📏 max_component: {max_component_size:.3f}m | min_component: {min_component_size:.3f}m {'✅' if dim_match else '❌'}\n"
        # )
        
        return dim_match

    def transform_to_map(self, x, z, stamp):
        # Create a PointStamped message for the input coordinates
        point_in = PointStamped()
        point_in.header.frame_id = 'camera_depth_optical_frame'  # Input frame
        point_in.header.stamp = stamp  # Timestamp of the original point cloud
        point_in.point = Point(x=x, y=0.09, z=z)  # Set the point coordinates

        try:
            # Lookup the transform from camera_depth_optical_frame to map
            transform = self.tfBuffer.lookup_transform(
                'map',  # Target frame
                point_in.header.frame_id,  # Source frame
                point_in.header.stamp,  # Time of the transform
                rclpy.duration.Duration(seconds=1.0)  # Timeout
            )

            # Transform the point to the map frame
            point_out = do_transform_point(point_in, transform)

            # Extract the transformed coordinates
            x_transformed = point_out.point.x
            y_transformed = point_out.point.y
            z_transformed = point_out.point.z

            return x_transformed, y_transformed, z_transformed

        except TransformException as e:
            self.get_logger().error(f"Failed to transform coordinates: {e}")
            return None, None, None


    def create_object(self, id, x, z, angle, stamp):

        x_map, y_map, _ = self.transform_to_map(x, z, stamp)

        if x_map is None or y_map is None:
            return

        # Map object type to a label
        if id == 'cube':
            type = 'OBJECT'
            category = 1
        elif id == 'sphere':
            type = 'OBJECT'
            category = 2
        elif id == 'plushie':
            type = 'OBJECT'
            category = 3
        else: # id == 'box':'
            type = 'BOX'
            category = 0 # Undefined cat
            self.detected_boxes.append((x_map, y_map))  # Store the detected box coordinates     
                   
        # Publish the detection message
        detection_msg = DetectionMsg()
        detection_msg.type = type  # "OBJECT" or "BOX"
        detection_msg.cat = category  # Category (1, 2, 3, etc.)
        detection_msg.x = x_map
        detection_msg.y = y_map
        detection_msg.theta = angle if type == 'BOX' else 0.0  # Orientation for BOX

        self.get_logger().info(f"Object: {type} | X: {x_map:.3f}m, Y: {y_map:.3f}")

        self.detection_publisher.publish(detection_msg)

        


    def classify_based_on_floor_contact(self, cluster_points, middle_layer_range=0.02, top_layer_range=0.005):
            cluster_points = np.array(cluster_points)        # Dynamically calculate the middle layer of the cluster
            # The middle layer is defined as points within a small range around the median height of the cluster
            median_height = np.median(cluster_points[:, 1])  # Median height of the cluster
            middle_layer_points = cluster_points[
                (cluster_points[:, 1] >= median_height - middle_layer_range) &
                (cluster_points[:, 1] <= median_height + middle_layer_range)
            ]
            num_middle_layer_points = len(middle_layer_points)        # Dynamically calculate the highest layer of the cluster
            min_height = np.min(cluster_points[:, 1])  # Maximum height of the cluster
            highest_layer_points = cluster_points[
                (cluster_points[:, 1] >= min_height) &
                (cluster_points[:, 1] <= min_height + top_layer_range)
            ]

            num_highest_layer_points = len(highest_layer_points)        # Calculate the ratio of middle layer points to highest layer points
            if num_highest_layer_points == 0:
                return "unknown"  # Avoid division by zero 
            


            ratio = num_middle_layer_points / num_highest_layer_points      
            #self.get_logger().info(f"ratio: {ratio}")        # Classification based on the ratio


            if 1 < ratio <= 6.5:  # Cube: ratio is approximately 1
                return "cube"
            elif 14 > ratio > 6.5:  # Sphere: middle layer has significantly more points
                return "sphere"
            else:
                return "unknown"  # Undefined object
    


    def dbscan(self, points, eps=0.05, min_samples=200):
        if len(points) == 0:
            return [], np.array([])

        db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        return db.labels_
    


    def publish_clusters(self, points, labels, original_header):
        if len(points) == 0:
            return
    
        # Filter out noise points (labels == -1)
        valid_mask = labels != -1
        filtered_points = points[valid_mask]
        filtered_labels = labels[valid_mask]
    
        # Return if no valid points
        if len(filtered_points) == 0:
            return
    
        # Assign a unique color to each cluster
        unique_colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (128, 0, 0), (0, 128, 0), (0, 0, 128)
        ]
    
        # Prepare data for PointCloud2
        cloud_data = []
        for i, (x, y, z) in enumerate(filtered_points):
            cluster_id = filtered_labels[i]
            color = unique_colors[int(cluster_id) % len(unique_colors)]
    
            r, g, b = color
            rgb = struct.unpack('f', struct.pack('BBBB', b, g, r, 0))[0]  # Pack into float
    
            cloud_data.append((x, y, z, rgb))
    
        # Define the PointCloud2 message fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
    
        # Create a new PointCloud2 message with the filtered points and colors
        cluster_msg = create_cloud(original_header, fields, cloud_data)
    
        # Publish the clusters
        self.cluster_publisher.publish(cluster_msg)
    



    # v ------------------ NOT REVIEWED BELOW ------------------ v


    def estimate_box_orientation(self, cluster_points):
        # Find the point with the lowest Z-coordinate
        lowest_z_point = cluster_points[np.argmin(cluster_points[:, 2])]
        
        # Find the point with the highest Z-coordinate
        highest_z_point = cluster_points[np.argmax(cluster_points[:, 2])]
        
        # Calculate the vector between the highest and lowest Z-points
        box_vector = highest_z_point - lowest_z_point
        
        # Normalize the vector
        box_vector_normalized = box_vector / np.linalg.norm(box_vector)
        
        # Fixed x-axis in the reference frame
        x_axis = np.array([1, 0, 0])
        
        # Compute the dot product and magnitude to calculate the angle
        dot_product = np.dot(x_axis, box_vector_normalized)
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Clip to handle floating-point precision
        
        # Convert to degrees
        angle_deg = np.degrees(angle)
        

        # Compute the dimensions of the box
        min_coords = np.min(cluster_points, axis=0)
        max_coords = np.max(cluster_points, axis=0)
        dimensions = max_coords - min_coords
        length, width, height = sorted(dimensions, reverse=True)
        
        # Check if the box is aligned with the X-axis
        if length < 0.17:  # Tolerance for floating-point comparison
            angle_deg = 90.0  # Rotate by 90 degrees to align with the long edge
        elif 0.17 < length < 0.25:
            angle_deg = 0.0
        
        self.get_logger().info(f"angle: {angle_deg}")

        return angle_deg



    def is_box(self, cluster_points):
        # Expected box dimensions (meters)
        EXPECTED_LENGTH = 0.24
        EXPECTED_WIDTH = 0.16
        EXPECTED_HEIGHT = 0.10
        TOLERANCE = 0.03  # 3 cm tolerance

        # 1. Calculate TRUE VERTICAL HEIGHT (y-axis)
        y_values = cluster_points[:, 1]
        height = np.max(y_values) - np.min(y_values)
        height_ok = abs(height - EXPECTED_HEIGHT) < TOLERANCE

        # 2. Calculate HORIZONTAL DIMENSIONS (X-Z plane)
        xz_points = cluster_points[:, [0,2]]
        pca = PCA(n_components=2)
        pca.fit(xz_points)
        
        # Project points onto horizontal principal axes
        projected = xz_points @ pca.components_.T
        max_size = np.ptp(projected[:, 0])  # Primary horizontal dimension
        min_size = np.ptp(projected[:, 1])   # Secondary horizontal dimension

        # 3. Match dimensions to expected length/width
        dim_match = (
            (abs(max_size - EXPECTED_LENGTH) < TOLERANCE) or
            (abs(max_size - EXPECTED_WIDTH) < TOLERANCE) or
            (abs(min_size - EXPECTED_WIDTH) < TOLERANCE)
        )

        # 4. Aspect ratio validation
        expected_aspect_1 = EXPECTED_LENGTH / EXPECTED_HEIGHT 
        expected_aspect_2 = EXPECTED_WIDTH / EXPECTED_HEIGHT 
        actual_aspect = max_size / height
        aspect_ok = abs(actual_aspect - expected_aspect_1) < 0.2 or abs(actual_aspect - expected_aspect_2) < 0.2

        # Debug output
        # self.get_logger().info(
        #     f"📏 Vertical Height (Z): {height:.3f}m | {'✅' if height_ok else '❌'}\n"
        #     f"📐 Horizontal Dimensions: L={max_size:.3f}m, W={min_size:.3f}m\n"
        #     f"🎯 Expected: L={EXPECTED_LENGTH}m, W={EXPECTED_WIDTH}m\n"
        #     f"🔍 Dim Match: {dim_match} | Aspect Ratio: {actual_aspect:.2f} ({aspect_ok})"
        # )

        return height_ok and (dim_match or aspect_ok)



def main(args=None):
    rclpy.init(args=args)

    examine_image = PointCloudDetection()

    try:
        rclpy.spin(examine_image)
    except KeyboardInterrupt:
        pass

    examine_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
