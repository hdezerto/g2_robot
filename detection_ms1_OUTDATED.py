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


class ExamineImage(Node):

    def __init__(self):
        super().__init__('examine_image')

        self.mat = None

        self.get_logger().info(f"Init detection")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer,self)

        self.sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            100)
       
        self.sub2 = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            100)
       
        self.pub = self.create_publisher(PointCloud2,'camera/camera_depth/color/points_transformed',100) 

        folder_path = os.path.expanduser('~/dd2419_ws/maps')
        file_name = 'map.txt'
        self.file_path = os.path.join(folder_path, file_name)

        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        with open(self.file_path, 'w') as file:
            file.write(f"")

        self.cluster_publisher = self.create_publisher(PointCloud2, 'clusters', 10)

    def image_callback(self, msg):
        sz = (msg.height, msg.width)
        if False:
            print('{encoding} {width} {height} {step} {data_size}'.format(
                encoding=msg.encoding, width=msg.width, height=msg.height,
                step=msg.step, data_size=len(msg.data)))
        if msg.step * msg.height != len(msg.data):
            print('bad step/height/data size')
            return

        if msg.encoding == 'rgb8':
            dirty = (self.mat is None or msg.width != self.mat.shape[1] or
                     msg.height != self.mat.shape[0] or len(self.mat.shape) < 2 or
                     self.mat.shape[2] != 3)
            if dirty:
                self.mat = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
            self.mat[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
            self.mat[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
            self.mat[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)
        elif msg.encoding == 'mono8':
            self.mat = np.array(msg.data).reshape(sz)
        else:
            print('unsupported encoding {}'.format(msg.encoding))
            return
        # if self.mat is not None:
        #     cv2.imshow('image', self.mat)
        #     cv2.waitKey(5)

    def cloud_callback(self, msg: PointCloud2):
        # Transform point cloud to 'map' frame
        frame_id = msg.header.frame_id
        target_frame = "map"  # Change this if needed

        try:
            t = self.tfBuffer.lookup_transform(target_frame, frame_id, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f'Could not transform {frame_id} to {target_frame}: {ex}')
            return

        transformed_points = []
        gen = pc2.read_points_numpy(msg, skip_nans=True)

        for p in gen[:, :3]:  # Extract XYZ
            point_stamped = PointStamped()
            point_stamped.header.frame_id = frame_id
            point_stamped.point.x, point_stamped.point.y, point_stamped.point.z = p

            transformed_point = do_transform_point(point_stamped, t)
            transformed_points.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])

        transformed_points = np.array(transformed_points)

        #################################################################
        points = gen[:, :3]
        colors = np.empty(points.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        colors = colors.astype(np.float32) / 255
        
        max_dist = 0.9
        distance = np.linalg.norm(transformed_points, axis=1)
        mask = (distance < max_dist) & (transformed_points[:,2] >= 0.01) & (transformed_points[:,2] <= 0.1)
        points = transformed_points[mask] 
        colors = colors[mask]

        # Step 1: Detect Clusters using DBSCAN
        clusters, labels = self.dbscan(points, eps=0.05, min_samples=200)

        self.publish_clusters(points, labels)

        if colors.shape[0] == 0:
            return

        # Convert RGB to HSV
        rgb_colors = colors * 255  # Scale back to 0-255a
        hsv_colors = cv2.cvtColor(rgb_colors.reshape(1, -1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV).reshape(-1, 3)

        # Define HSV ranges for filtering
        lower_red, upper_red = np.array([2, 230, 95]), np.array([2, 240, 100])
        lower_green1, upper_green1 = np.array([81,100,44]), np.array([84,255,105])
        lower_green2, upper_green2 = np.array([73,210,90]), np.array([74,240,120])
        lower_blue, upper_blue = np.array([99, 254, 75]), np.array([99, 255, 80])

        # Create masks
        red_mask = ((hsv_colors[:, 0] >= lower_red[0]) & (hsv_colors[:, 0] <= upper_red[0]))
        green_mask = ((hsv_colors[:, 0] >= lower_green1[0]) & (hsv_colors[:, 0] <= upper_green1[0]))  | \
                     ((hsv_colors[:, 0] >= lower_green2[0]) & (hsv_colors[:, 0] <= upper_green2[0]))
        blue_mask = ((hsv_colors[:, 0] >= lower_blue[0]) & (hsv_colors[:, 0] <= upper_blue[0]))

        # Apply masks
        red_points = points[red_mask]
        green_points = points[green_mask]
        blue_points = points[blue_mask]
        

        if len(clusters) > 0:
                centroid = np.mean(points, axis=0)  
                x, y, z = centroid  
                self.create_object('cube',x,y,0.0)
                self.get_logger().info('Object!')
        else:
            self.get_logger().info('NOT OBJECT')
        # if len(red_points) > 400 or len(green_points) > 400 or len(blue_points)>400:
        #         self.get_logger().info('Object!')
        # elif self.is_box(points):
        #     self.get_logger().info('Object!')
        # elif self.is_plushie(points):
        #     self.get_logger().info('Object!')
        # else:
        #     self.get_logger().info('NOT OBJECT')
 
    def is_box(self, points_filtered):
        #Detect if the object is a box based on dimensions.
        min_coords = np.min(points_filtered, axis=0)
        max_coords = np.max(points_filtered, axis=0)
        dimensions = max_coords - min_coords
        length, width, height = sorted(dimensions, reverse=True)

        #self.get_logger().info(f"Box Check - length: {length:.3f}, width: {width:.3f}, height: {height:.3f}")

        # Check box dimensions (with tolerance)
        return (0.23 <= length and 0.15 <= width and 0.09 <= height <= 0.1) 
    
    def is_plushie(self, points_filtered):
        #Detect if the object is a box based on dimensions.
        min_coords = np.min(points_filtered, axis=0)
        max_coords = np.max(points_filtered, axis=0)
        dimensions = max_coords - min_coords
        length, width, height = sorted(dimensions, reverse=True)

        #self.create_object("plushie", 0, 0, 0)

        #self.get_logger().info(f"Plushie Check - length: {length:.3f}, width: {width:.3f}, height: {height:.3f}")

        # Check box dimensions (with tolerance)
        return (0.06 <= length <= 0.1 and 0.03 <= width and height <= 0.9)
    

    def dbscan(self, points, eps=0.05, min_samples=200):
        if len(points) == 0:
            return [], np.array([])

        tree = cKDTree(points)
        labels = -np.ones(len(points))  # -1 means unclassified
        cluster_id = 0

        for i in range(len(points)):
            if labels[i] != -1:
                continue  # Already classified

            neighbors = tree.query_ball_point(points[i], eps)

            if len(neighbors) < min_samples:
                continue  # Noise, not a cluster

            labels[i] = cluster_id
            queue = list(neighbors)

            while queue:
                point_idx = queue.pop()

                if labels[point_idx] == -1:  # If noise, mark as part of the cluster
                    labels[point_idx] = cluster_id
                if labels[point_idx] != -1:
                    continue  # Already assigned to a cluster

                labels[point_idx] = cluster_id
                new_neighbors = tree.query_ball_point(points[point_idx], eps)

                if len(new_neighbors) >= min_samples:
                    queue.extend(new_neighbors)  # Expand cluster

            cluster_id += 1

        clusters = []
        for i in range(cluster_id):
            clusters.append(points[labels == i])

        return clusters, labels  # Return both clusters and their labels

    def publish_clusters(self, points, labels):

        if len(points) == 0:
            return

        cluster_msg = PointCloud2()
        cluster_msg.header.stamp = self.get_clock().now().to_msg()
        cluster_msg.header.frame_id = "map"  # Adjust to your robot's frame

        cluster_msg.height = 1
        cluster_msg.width = len(points)
        cluster_msg.is_dense = False
        cluster_msg.is_bigendian = False
        cluster_msg.point_step = 16  # 4 floats (x, y, z, color)
        cluster_msg.row_step = cluster_msg.point_step * len(points)

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
        for i, (x, y, z) in enumerate(points):
            cluster_id = labels[i]
            if cluster_id == -1:  # Noise points in black
                color = (0, 0, 0)
            else:
                color = unique_colors[int(cluster_id) % len(unique_colors)]

            r, g, b = color
            rgb = struct.unpack('f', struct.pack('BBBB', b, g, r, 0))[0]  # Pack into float

            cloud_data.append(struct.pack('ffff', x, y, z, rgb))

        cluster_msg.data = b''.join(cloud_data)
        
        # Publish the clusters
        self.cluster_publisher.publish(cluster_msg)

    def create_object(self,type,x,y,angle):
        
        if type == 'cube': L = 1
        elif type == 'sphere': L = 2
        elif type == 'plushie': L = 3
        elif type == 'box': L = 'B'
        else: L = 'Undefined'

        with open(self.file_path, 'a') as file:
            file.write(f"{L} {x} {y} {angle} \n")

        self.get_logger().info(f"Created object: {L} at postion:({x},{y})")

        return None


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