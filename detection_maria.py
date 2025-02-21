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

from sensor_msgs_py.point_cloud2 import create_cloud # Convert colors to a single float value representing RGB

class ExamineImage(Node):

    def __init__(self):
        super().__init__('examine_image')

        self.mat = None

        self.get_logger().info(f"Init detection")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.sub2 = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            100)

        #self.pub = self.create_publisher(PointCloud2, 'camera/camera_depth/color/points_transformed', 100) # HUGO CHECK
        self.pub = self.create_publisher(PointCloud2, '/depth_points_filtered', 100)

        # HUGO EDITED
        folder_path = os.path.join(os.getcwd(), 'maps') # current directory + /maps
        
        # HUGO ADDED
        # Create the 'maps' folder if it doesn't exist
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
        
        file_name = 'Map.txt'
        self.file_path = os.path.join(folder_path, file_name)
        with open(self.file_path, 'w') as file:
            file.write(f"")

        self.cluster_publisher = self.create_publisher(PointCloud2, 'clusters', 10)

        self.marker_publisher = self.create_publisher(MarkerArray, 'map_objects', 10)
        self.timer = self.create_timer(2.0, self.publish_map_objects)  # Update every 2 seconds

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
        # Transform point cloud to 'map' frame
        frame_id = msg.header.frame_id
        target_frame = "map"  # Change this if needed

        gen = pc2.read_points_numpy(msg, skip_nans=True)
        
        # try:
        #     t = self.tfBuffer.lookup_transform(target_frame, frame_id, rclpy.time.Time())
        # except TransformException as ex:
        #     self.get_logger().error(f'Could not transform {frame_id} to {target_frame}: {ex}')
        #     return

        # transformed_points = []
        

        # for p in gen[:, :3]:  # Extract XYZ
        #     point_stamped = PointStamped() # HUGO CHECK EFFICIENCY
        #     point_stamped.header.frame_id = frame_id # HUGO CHECK EFFICIENCY
        #     point_stamped.point.x, point_stamped.point.y, point_stamped.point.z = p

        #     transformed_point = do_transform_point(point_stamped, t)
        #     transformed_points.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

        # transformed_points = np.array(transformed_points)
        
        points = gen[:, :3] # XYZ coordinates in camera_depth_optical_frame
        colors = np.empty(points.shape, dtype=np.uint32)

        # Extract RGB colors from the point cloud
        for idx, x in enumerate(gen):
            c = x[3] # extract color floating-point value
            s = struct.pack('>f', c) # pack float into bytes
            i = struct.unpack('>l', s)[0] # convert byte string into a long integer
            pack = ctypes.c_uint32(i).value # convert to an unsigned 32-bit integer
            # | 31-24 | 23-16 | 15-8 | 7-0 |
            # | Alpha |  Red  | Green | Blue |
            # dtype=np.uint8 converts the isolated component to unsigned 8-bit integer
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8) # Red
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8) # Green
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8) # Blue
        
        colors = colors.astype(np.float32) / 255 # Convert to 32-bit floating-point numbers and normalize to [0, 1]

        max_dist = 0.9
        distance = np.linalg.norm(points, axis=1)
        mask = (distance < max_dist) & (points[:, 2] >= 0.005) & (points[:, 2] <= 0.9) 
        filtered_points = points[mask] # Select points within the distance range
        filtered_colors = colors[mask] # Select colors within the distance range

        # | corresponds to bitwise OR
        rgb_colors = (filtered_colors[:, 0] * 255).astype(np.uint32) << 16 | \
                     (filtered_colors[:, 1] * 255).astype(np.uint32) << 8 | \
                     (filtered_colors[:, 2] * 255).astype(np.uint32)
        rgb_colors = rgb_colors.view(np.float32).reshape(-1, 1)        
        points_with_colors = np.hstack((filtered_points, rgb_colors)) # Combine points and colors
        header = msg.header
        # offset is in bytes. count is the number of elements in the field.
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        filtered_cloud_msg = create_cloud(header, fields, points_with_colors) # Publish the PointCloud2 message
        self.pub.publish(filtered_cloud_msg)




        """
        # Perform spatial clustering using DBSCAN
        labels = self.dbscan(filtered_points, eps=0.05, min_samples=300)

        
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
            cluster_points = filtered_points[cluster_mask]
            cluster_colors = filtered_colors[cluster_mask]

            # HUGO: why is this needed?
            # if cluster_colors.shape[0] == 0:
            #     self.get_logger().info(f'Cluster {cluster_label} has no valid colors. Skipping.')
            #     continue

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

            pure_red = pure_green = pure_blue = False
            
            # HUGO CHECK: i dont understand why we need the 10 limit
            if len(red_points) > 300 and len(green_points) < 10 and len(blue_points) < 10:
                pure_red = True
            if len(red_points) < 10 and len(green_points) > 300 and len(blue_points) < 10:
                pure_green = True
            if len(red_points) < 10 and len(green_points) < 10 and len(blue_points) > 300:
                pure_blue = True

            # Classify based on floor contact points for the current cluster
            object_type = self.classify_based_on_floor_contact(cluster_points)

            if pure_red or pure_green or pure_blue:
                centroid = np.mean(cluster_points, axis=0)
                x, y, z = centroid

                if object_type == "sphere":
                    self.get_logger().info(f'🔵 Cluster {cluster_label} is a sphere!')
                    self.create_object('sphere', x + 0.01, y, 0.0)
                elif object_type == "cube":
                    self.get_logger().info(f'🟥 Cluster {cluster_label} is a cube!')
                    self.create_object('cube', x, y, 0.0)

            
            
            # HUGO: STILL TO TEST BELLOW

            # elif self.is_box(cluster_points):  # If detected object is a box
            #     self.get_logger().info(f'📦 Cluster {cluster_label} is a box!')

            #     # Compute the orientation angle of the box
            #     angle = self.estimate_box_orientation(cluster_points)

            #     # Store box with angle information
            #     centroid = np.mean(cluster_points, axis=0)
            #     x, y, z = centroid
            #     self.create_object('box', x + 0.08, y, angle)

            # elif self.is_plushie(cluster_points):
            #     self.get_logger().info(f'🧸 Cluster {cluster_label} is a plushie!')
            #     centroid = np.mean(cluster_points, axis=0)
            #     x, y, z = centroid
            #     self.create_object('plushie', x + 0.01, y, 0.0)

            else:
                self.get_logger().info(f'Cluster {cluster_label} is NOT a recognized object.')
            """



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



    def estimate_box_orientation(self, cluster_points):
        # Compute PCA to find the principal axis
        pca = PCA(n_components=2)
        pca.fit(cluster_points[:, :2])  # Only consider x and y coordinates

        # The first principal component is the direction of the longest axis
        principal_axis = pca.components_[0]

        # Compute the angle between the principal axis and the x-axis
        angle = np.arctan2(principal_axis[1], principal_axis[0])

        # Convert to degrees and ensure it's within 0-180 degrees
        angle_deg = np.degrees(angle) % 180

        # Ensure the angle is relative to the long edge
        if angle_deg > 90:
            angle_deg -= 180  # Adjust to -90 to 90 degrees

        # Compute the dimensions of the box
        min_coords = np.min(cluster_points, axis=0)
        max_coords = np.max(cluster_points, axis=0)
        dimensions = max_coords - min_coords
        length, width, height = sorted(dimensions, reverse=True)

        # Check if the principal axis corresponds to the long edge
        # If the width is closer to the long edge (0.23), swap the angle
        if width < 0.18 and length < 0.25:  # Tolerance for floating-point comparison
            angle_deg = 0.0  # Rotate by 90 degrees to align with the long edge
        elif width > 0.23:
            angle_deg = 90.0
        else:
            # Ensure the angle is within -90 to 90 degrees
            angle_deg = angle_deg % 180
            if angle_deg > 90:
                angle_deg -= 180

            # Invert the angle for consistency with your coordinate system
            angle_deg = -angle_deg

        return angle_deg



    # DIOGO SAID IT'S NOT WORKING 
    def is_box(self, points_filtered):
        # Detect if the object is a box based on dimensions.
        min_coords = np.min(points_filtered, axis=0)
        max_coords = np.max(points_filtered, axis=0)
        dimensions = max_coords - min_coords
        length, width, height = sorted(dimensions, reverse=True)

        #self.get_logger().info(f"Box Check - length: {length:.3f}, width: {width:.3f}, height: {height:.3f}")

        # Check box dimensions (with tolerance)
        return (0.23 <= length and 0.15 <= width and 0.09 <= height <= 0.1)



    def is_plushie(self, points_filtered):
        # Detect if the object is a box based on dimensions.
        min_coords = np.min(points_filtered, axis=0)
        max_coords = np.max(points_filtered, axis=0)
        dimensions = max_coords - min_coords
        length, width, height = sorted(dimensions, reverse=True)

        # self.get_logger().info(f"Plushie Check - length: {length:.3f}, width: {width:.3f}, height: {height:.3f}")

        # Check box dimensions (with tolerance)
        return (0.06 <= length <= 0.12 and 0.03 <= width and height <= 0.9)



    def classify_based_on_floor_contact(self, cluster_points, floor_threshold=0.01):
        cluster_points = np.array(cluster_points)

        # if cluster_points.shape[0] == 0:
        #     return "unknown"

        # Find points that are very close to the ground (y ≈ 0)
        floor_contact_points = cluster_points[np.abs(cluster_points[:, 2]) < floor_threshold]
        num_floor_contacts = len(floor_contact_points)

        # self.get_logger().info(f"📊 Floor Contact Points: {num_floor_contacts} (Threshold: {contact_threshold})")

        # Classification based on contact point count
        if num_floor_contacts > 205 and num_floor_contacts < 300:
            #   self.get_logger().info("✅ Object is a Cube (many floor contact points)")
            return "cube"
        else:
            #  self.get_logger().info("🔵 Object is a Sphere (few floor contact points)")
            return "sphere"



    # HUGO CHECK EFFICIENCY
    def dbscan(self, points, eps=0.05, min_samples=200):
        if len(points) == 0:
            return [], np.array([])

        db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = db.labels_
        #clusters = [points[labels == i] for i in np.unique(labels) if i != -1]
        return labels



    def publish_clusters(self, points, labels):
        if len(points) == 0:
            return

        # Filter out noise points (labels == -1)
        valid_mask = labels != -1
        filtered_points = points[valid_mask]
        filtered_labels = labels[valid_mask]

        # Return if no valid points
        if len(filtered_points) == 0:
            return

        # Create PointCloud2 message
        cluster_msg = PointCloud2()
        cluster_msg.header.stamp = self.get_clock().now().to_msg()
        cluster_msg.header.frame_id = "map"  # Adjust to your robot's frame

        cluster_msg.height = 1 # Single row of points
        cluster_msg.width = len(filtered_points) # Number of points
        cluster_msg.is_dense = False
        cluster_msg.is_bigendian = False # Little-endian byte order
        cluster_msg.point_step = 16  # 4 x 4 bytes  floats (x, y, z, color)
        cluster_msg.row_step = cluster_msg.point_step * len(filtered_points) # Total bytes

        # Define PointCloud2 fields (XYZ + RGB)
        cluster_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

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

            cloud_data.append(struct.pack('ffff', x, y, z, rgb))

        cluster_msg.data = b''.join(cloud_data)

        # Publish the clusters
        self.cluster_publisher.publish(cluster_msg)



    def create_object(self, type, x, y, angle):
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

        # Format the new object entry
        new_entry = f"{L} {x:.2f} {y:.2f} {angle:.1f}\n"

        # Check if the file exists and read its content
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as file:
                existing_entries = file.readlines()
        else:
            existing_entries = []

        # Check if the new entry already exists in the file
        if new_entry not in existing_entries:
            # Append the new entry to the file
            with open(self.file_path, 'a') as file:
                file.write(new_entry)
            #  self.get_logger().info(f"Created object: {L} at position: ({x:.2f}, {y:.2f})")
        # else:
        #   self.get_logger().info(f"Object already exists at position: ({x:.2f}, {y:.2f})")

        return None



    def publish_map_objects(self):
        marker_array = MarkerArray()

        # Read the Map.txt file
        if not os.path.exists(self.file_path):
            return

        with open(self.file_path, 'r') as file:
            lines = file.readlines()

        # Clear previous markers (optional but recommended)
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        # Parse each line and create markers
        for idx, line in enumerate(lines):
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
            marker.id = idx

            # Set a unique namespace to avoid duplication in the same MarkerArray
            marker.ns = "object_{}".format(idx)

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

            # Adjust scale for 2D square (x and y scale) and a small height (z scale) to make it a square, not a cube
            if obj_type == '1':  # Square (using CUBE type with adjusted scale)
                marker.type = Marker.CUBE
                marker.scale.x = 0.04  # Width of the square
                marker.scale.y = 0.04  # Height of the square
                marker.scale.z = 0.01  # Minimal height (so it looks like a 2D object)
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif obj_type == '2':  # Circle (using SPHERE type, but we keep z = 0)
                marker.type = Marker.SPHERE
                marker.scale.x = 0.04  # Diameter of the circle
                marker.scale.y = 0.04  # Diameter of the circle
                marker.scale.z = 0.01  # Minimal height (so it looks like a 2D object)
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif obj_type == '3':  # Plushie (but we keep z = 0)
                marker.type = Marker.CUBE
                marker.scale.x = 0.06  # Width of the square
                marker.scale.y = 0.12  # Height of the square
                marker.scale.z = 0.01  # Minimal height (so it looks like a 2D object)
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif obj_type == 'B':  # Box (can still use CUBE type, just as a 2D object)
                marker.type = Marker.CUBE
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0  # Fully opaque

                # Original dimensions of the box
                original_width = 0.16
                original_length = 0.24

                # Convert the angle from degrees to radians
                angle_rad = np.radians(angle)

                # Compute effective dimensions based on the angle
                effective_width = (
                    original_width * abs(np.cos(angle_rad)) +
                    original_length * abs(np.sin(angle_rad))
                )
                effective_length = (
                    original_width * abs(np.sin(angle_rad)) +
                    original_length * abs(np.cos(angle_rad))
                )

                # Set the scale of the marker
                marker.scale.x = effective_width  # Width of the square
                marker.scale.y = effective_length  # Height of the square
                marker.scale.z = 0.01  # Minimal height (so it looks like a 2D object)


            marker.color.a = 1.0  # Alpha (opacity)
            marker.lifetime.sec = 5  # Persist indefinitely

            marker_array.markers.append(marker)

        # Publish the markers
        self.marker_publisher.publish(marker_array)



def main(args=None):
    rclpy.init(args=args)

    examine_image = ExamineImage()

    try:
        rclpy.spin(examine_image)
    except KeyboardInterrupt:
        pass

    examine_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()