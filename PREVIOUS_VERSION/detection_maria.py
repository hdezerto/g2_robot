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

class PointCloudDetection(Node):

    def __init__(self):
        super().__init__('PointCloudDetection_node')

        self.mat = None

        self.get_logger().info(f"Init PointCloudDetection node")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

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

        folder_path = os.path.join(os.getcwd(), 'maps') # current directory + /maps
        
        # Create the 'maps' folder if it doesn't exist
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        
        file_name = 'Map.txt'
        self.file_path = os.path.join(folder_path, file_name)
        with open(self.file_path, 'w') as file:
            file.write(f"")

        self.cluster_publisher = self.create_publisher(PointCloud2, '/clusters', 10)

        #self.marker_publisher = self.create_publisher(MarkerArray, '/map_objects', 10)
        
        #self.timer = self.create_timer(2.0, self.publish_map_objects)  # Update every 2 seconds

        # Add a counter to track the number of messages received
        self.message_counter = 0

        self.last_line_count = 0

        # Initialize the list of existing entries to avoid always reading the file
        self.existing_entries = []


        # # Publisher for the workspace perimeter marker
        # self.workspace_publisher = self.create_publisher(Marker, 'workspace_perimeter', 10)

        # # Define workspace vertices (in centimeters, converted to meters)
        # self.workspace_vertices = [
        #     (-220 / 100, -130 / 100),
        #     (220 / 100, -130 / 100),
        #     (450 / 100, 66 / 100),
        #     (700 / 100, 66 / 100),
        #     (700 / 100, 284 / 100),
        #     (546 / 100, 284 / 100),
        #     (546 / 100, 130 / 100),
        #     (-220 / 100, 130 / 100)
        # ]

        # # Publish the workspace perimeter
        # self.publish_workspace_perimeter()



    def cloud_callback(self, msg: PointCloud2):

        # Increment the message counter
        self.message_counter += 1
        # Only process every 5 messages. Fequency of /camera/camera/depth/color/points is around 15 point clouds per second
        if self.message_counter % 2 != 0:
            return
        # Reset the counter to avoid overflow
        self.message_counter = 0

        start_time = time.time() # DEBUGGING EFFICIENCY

        # Read point cloud data from the message
        points_data = pc2.read_points_numpy(msg, skip_nans=True)

        # Extract XYZ coordinates from the point cloud
        points = points_data[:, :3]  # Shape (N, 3)

        # Set distance threshold for filtering
        max_dist = 0.9 # Maximum distance from the sensor (in meters)
        
        # Compute Euclidean distance of each point from the origin
        distances = np.linalg.norm(points, axis=1)

        # Create a boolean mask to filter points:
        # - Points within max_dist from the sensor
        # - Points above the floor (0.01 < y < 0.085) (y-axis points downwards)
        mask = (distances < max_dist) & (points[:, 1] < 0.085) & (0.01 < points[:, 1]) # TEST: maybe more floor can be removed by decreasing the y value
        
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


        # v ----------------- CAN BE COMMENTED OUT AFTER TESTING ----------------- v
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
        # ^ ----------------------------------------------------------------- ^

        
        # Perform spatial clustering using DBSCAN
        labels = self.dbscan(points, eps=0.05, min_samples=300) # CHECK
        
        # Publish clusters using the original header
        self.publish_clusters(points, labels, msg.header) # CAN BE COMMENTED OUT AFTER TESTING

        valid_labels = labels[labels != -1] # Filter out noise points (label = -1)
        unique_labels = np.unique(valid_labels)

        # Define HSV [Hue (0-179), Saturation(0-255), Value(0-255)] ranges for filtering color ranges for red, green, and blue
        lower_red, upper_red = np.array([2, 230, 95]), np.array([2, 240, 100])
        lower_green1, upper_green1 = np.array([81, 100, 44]), np.array([84, 255, 105])
        lower_green2, upper_green2 = np.array([73, 210, 90]), np.array([74, 240, 120])
        lower_blue, upper_blue = np.array([99, 254, 75]), np.array([99, 255, 80])
        lower_brown, upper_brown =np.array([15, 68, 137]), np.array([17, 76, 134])

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
            brown_mask = (hsv_colors[:, 0] >= lower_brown[0]) & (hsv_colors[:, 0] <= upper_brown[0])

            # Apply masks for the current cluster
            red_points = cluster_points[red_mask]
            green_points = cluster_points[green_mask]
            blue_points = cluster_points[blue_mask]
            brown_points = cluster_points[brown_mask]

            # Calculate the total number of points in the cluster
            total_points = len(cluster_points)

            # Calculate the ratio of red, green, and blue points
            red_ratio = len(red_points) / total_points
            green_ratio = len(green_points) / total_points
            blue_ratio = len(blue_points) / total_points
            brown_ratio = len(brown_points) / total_points

            pure_red = pure_green = pure_blue = pure_brown = False
            
            # Check if the cluster is predominantly red, green, blue or brown
            if red_ratio > 0.01 and green_ratio == 0.0 and blue_ratio == 0.0:
                pure_red = True
            elif green_ratio > 0.01 and red_ratio == 0.0 and blue_ratio == 0.0:
                pure_green = True
            elif blue_ratio > 0.01 and red_ratio == 0.0 and green_ratio == 0.0:
                pure_blue = True
            elif brown_ratio > 0.01 and red_ratio == 0.0 and green_ratio == 0.0 and blue_ratio == 0.0:
                pure_brown = True
            
            x, y, z = np.mean(cluster_points, axis=0) # Calculate the centroid of the cluster

            # Classify based on floor contact points for the current cluster
            object_type = self.classify_based_on_floor_contact(cluster_points)

            if pure_brown:
                if object_type == "cube":
                    self.get_logger().info(f'🟫 Cluster {cluster_label} is a cube!')
                    self.create_object('cube', x, z + 0.02, 0.0, msg.header.stamp)

            if pure_red or pure_green or pure_blue:
                if object_type == "sphere":
                    if pure_red:
                        emoji = "🔴"  # Red circle emoji
                    elif pure_green:
                        emoji = "🟢"  # Green circle emoji
                    elif pure_blue:
                        emoji = "🔵"  # Blue circle emoji
                    self.get_logger().info(f'{emoji} Cluster {cluster_label} is a sphere!')
                    self.create_object('sphere', x, z + 0.02, 0.0, msg.header.stamp)
                elif object_type == "cube":
                    if pure_red:
                        emoji = "🟥"  # Red square emoji
                    elif pure_green:
                        emoji = "🟩"  # Green square emoji
                    elif pure_blue:
                        emoji = "🟦"  # Blue square emoji
                    self.get_logger().info(f'{emoji} Cluster {cluster_label} is a cube!')
                    self.create_object('cube', x, z + 0.02, 0.0, msg.header.stamp)
                elif object_type == "unknown":
                    self.get_logger().info(f'Object not identified :(!')

            elif self.is_plushie(cluster_points): # If detected object is a plushie
                self.get_logger().info(f'🧸 Cluster {cluster_label} is a plushie!')
                self.create_object('plushie', x, z + 0.01, 0.0, msg.header.stamp)

            elif self.is_box(cluster_points):  # If detected object is a box
                self.get_logger().info(f'📦 Cluster {cluster_label} is a box!')

                # Compute the orientation angle of the box
                angle = self.estimate_box_orientation(cluster_points)
                
                # Store box with angle information
                if angle == 0.0:
                    self.create_object('box', x, z + 0.08, angle, msg.header.stamp)
                elif angle == 90.0:
                    self.create_object('box', x, z + 0.12, angle, msg.header.stamp)
                else:
                    self.create_object('box', x, z + 0.08, angle, msg.header.stamp)

            else:
                self.get_logger().info(f'Cluster {cluster_label} is NOT a recognized object.')

            # ------------ TIMER FOR EFFICIENCY CHECK (move where desired) ------------
            end_time = time.time()
            self.get_logger().info(f"Processing time: {end_time - start_time:.4f} seconds")
            # ------------------------------------------------------------------------


    # v ------------------ FUNCTIONS BELOW ------------------ v

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



    def create_object(self, type, x, z, angle, stamp):
        # Map object type to a label
        if type == 'cube':
            L = 1
        elif type == 'sphere':
            L = 2
        elif type == 'plushie':
            L = 3
        elif type == 'box':
            L = 'B'
        else:
            L = 'Undefined'

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

            # Format the new object entry with transformed coordinates
            new_entry = f"{L} {x_transformed:.2f} {y_transformed:.2f} {angle:.1f}\n"

            # Check if the new entry is a duplicate based on proximity
            is_duplicate = False
            for entry in self.existing_entries:
                parts = entry.strip().split()
                if len(parts) < 4:
                    continue

                # Extract coordinates from the existing entry
                existing_x = float(parts[1])
                existing_y = float(parts[2])

                # Calculate Euclidean distance between the new and existing coordinates
                distance = np.sqrt((x_transformed - existing_x)**2 + (y_transformed - existing_y)**2)

                # If the distance is less than 0.01, consider it a duplicate
                if distance < 0.01:
                    is_duplicate = True
                    break

            # If not a duplicate, append the new entry to the file
            if not is_duplicate:
                with open(self.file_path, 'a') as file:
                    file.write(new_entry)
                self.existing_entries.append(new_entry)
                #self.get_logger().info(f"Created object: {L} at position: ({x_transformed:.2f}, {y_transformed:.2f})")
            #else:
                #self.get_logger().info(f"Object already exists near position: ({x_transformed:.2f}, {y_transformed:.2f})")

        except TransformException as e:
            self.get_logger().error(f"Failed to transform coordinates: {e}")     

        return None



    def classify_based_on_floor_contact(self, cluster_points, middle_layer_range=0.02, top_layer_range=0.005):
  
        # Extract Y-coordinates (height values)
        heights = cluster_points[:, 1]

        # Compute median height efficiently
        median_height = np.median(heights)

        # Compute the lowest height in the cluster
        min_height = np.min(heights)

        # Boolean masks for filtering layers
        middle_mask = (heights >= median_height - middle_layer_range) & (heights <= median_height + middle_layer_range)
        top_mask = (heights >= min_height) & (heights <= min_height + top_layer_range)

        # Count the number of points in each layer
        num_middle_layer_points = np.count_nonzero(middle_mask)
        num_highest_layer_points = np.count_nonzero(top_mask)

        """
        # Log the results for testing ratios
        self.get_logger().info(
            f"Middle Layer: {num_middle_layer_points}, "
            f"Highest Layer: {num_highest_layer_points}, "
            f"Ratio: {ratio:.2f}"
        )
         """
        # Avoid division by zero
        if num_highest_layer_points == 0:
            return "unknown"

        # Calculate ratio
        ratio = num_middle_layer_points / num_highest_layer_points

        # Classification based on the ratio
        if 1 < ratio <= 6:
            return "cube"
        elif 6 < ratio < 14:
            return "sphere"
        else:
            return "unknown"
    


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
        

        # HUGO CHECK: diego explain below please 
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
            (abs(max_size - EXPECTED_WIDTH) < TOLERANCE) or # HUGO CHECK: i dont understand this one 
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




    def publish_map_objects(self):
        marker_array = MarkerArray()

        # Read the Map.txt file
        if not os.path.exists(self.file_path):
            return

        with open(self.file_path, 'r') as file:
            lines = file.readlines()

        # Get the number of new lines added since the last read
        new_line_count = len(lines) - self.last_line_count

        # If no new lines, return
        if new_line_count <= 0:
            return

        # Clear all previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        # Process only the new lines
        for idx, line in enumerate(lines[-new_line_count:]):
            parts = line.strip().split()
            if len(parts) < 4:
                continue

            obj_type = parts[0]
            x = float(parts[1])
            y = float(parts[2])
            angle = float(parts[3])

            marker = Marker()
            marker.header.frame_id = "map"  # Map frame for 2D visualization
            marker.header.stamp = self.get_clock().now().to_msg()

            # Unique ID for each marker
            marker.id = self.last_line_count + idx  # Ensure unique IDs

            # Set a unique namespace to avoid duplication in the same MarkerArray
            marker.ns = "object_{}".format(self.last_line_count + idx)

            # Default to CUBE (for 2D square)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0  # z is always 0 for 2D

            # Set the orientation based on the angle
            q = Quaternion()
            q.z = np.sin(np.radians(angle) / 2.0)
            q.w = np.cos(np.radians(angle) / 2.0)
            marker.pose.orientation = q

            if obj_type == '1':  # Cube (using CUBE type with adjusted scale)
                marker.type = Marker.CUBE
                marker.scale.x = 0.04  # Width of the square
                marker.scale.y = 0.04  # Height of the square
                marker.scale.z = 0.01  # Minimal height (so it looks like a 2D object)
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif obj_type == '2':  # Sphere (using SPHERE type, but we keep z = 0)
                marker.type = Marker.SPHERE
                marker.scale.x = 0.04  # Diameter of the circle
                marker.scale.y = 0.04  # Diameter of the circle
                marker.scale.z = 0.01  # Minimal height (so it looks like a 2D object)
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif obj_type == '3':  # Plushie (using CUBE type, but we keep z = 0)
                marker.type = Marker.CUBE
                marker.scale.x = 0.06  # Width of the square
                marker.scale.y = 0.08  # Height of the square
                marker.scale.z = 0.01  # Minimal height (so it looks like a 2D object)
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif obj_type == 'B':  # Box (using CUBE type, just as a 2D object)
                # Original dimensions of the box
                original_width = 0.16
                original_length = 0.24

                # Set the scale of the marker to the original dimensions
                marker.scale.x = original_width  # Width of the box
                marker.scale.y = original_length  # Length of the box
                marker.scale.z = 0.01  # Minimal height (so it looks like a 2D object)

                # Convert the angle from degrees to radians
                angle_rad = np.radians(angle)

                # Set the orientation based on the angle
                q = Quaternion()
                q.z = np.sin(angle_rad / 2.0)  # Rotation around the Z-axis
                q.w = np.cos(angle_rad / 2.0)  # Quaternion scalar component
                marker.pose.orientation = q

                # Set the color
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0  # Fully opaque

            marker.color.a = 1.0  # Alpha (opacity)
            marker.lifetime.sec = 2  # Persist for 2 seconds

            marker_array.markers.append(marker)

        # Update the last line count
        self.last_line_count = len(lines)

        # Publish the markers
        self.marker_publisher.publish(marker_array)

    


    # def publish_workspace_perimeter(self):
    #     """
    #     Publish the workspace perimeter as a LINE_STRIP marker in RViz2.
    #     """
    #     marker = Marker()
    #     marker.header.frame_id = "map"  # Ensure this frame exists in your TF tree
    #     marker.header.stamp = self.get_clock().now().to_msg()  # Ensure current timestamp
    #     marker.ns = "workspace"
    #     marker.id = 0
    #     marker.type = Marker.LINE_STRIP
    #     marker.action = Marker.ADD

    #     # Set the scale of the lines (thickness)
    #     marker.scale.x = 0.05  # Increased line thickness for better visibility

    #     # Set the color of the lines (e.g., green)
    #     marker.color.r = 0.0
    #     marker.color.g = 1.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0  # Fully opaque

    #     # Add the vertices of the workspace polygon
    #     for x, y in self.workspace_vertices:
    #         point = Point()
    #         point.x = x  # Already in meters
    #         point.y = y  # Already in meters
    #         point.z = 0.0  # Workspace is on the ground (z = 0)
    #         marker.points.append(point)

    #     # Close the polygon by adding the first vertex again
    #     first_point = Point()
    #     first_point.x = self.workspace_vertices[0][0]
    #     first_point.y = self.workspace_vertices[0][1]
    #     first_point.z = 0.0
    #     marker.points.append(first_point)

    #     # Publish the marker
    #     self.workspace_publisher.publish(marker)






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