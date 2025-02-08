import math
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import tf2_geometry_msgs

import struct
from geometry_msgs.msg import TransformStamped, Point, Quaternion
from tf2_ros import TransformBroadcaster

class Detection(Node):

    def __init__(self):
        super().__init__('detection')

        # Initialize the publisher for filtered point cloud
        self._pub = self.create_publisher(PointCloud2, '/camera/depth/color/f_points', 10)

        # Initialize the TF broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to point cloud topic
        self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)

        # Initialize the transform listener and assign it a buffer (commented out for now)
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)


    def cloud_callback(self, msg: PointCloud2):
        """Processes the incoming point cloud and filters points."""
        self.get_logger().info(f"Something")
        # Convert ROS PointCloud2 -> NumPy array
        gen = pc2.read_points_numpy(msg, skip_nans=True)

        # Extract points (x, y, z) and colors
        points = gen[:, :3]  # x, y, z columns
        colors = gen[:, 3]  # color column (packed RGB format)

        # Unpack colors into RGB components
        unpacked_colors = np.zeros((colors.shape[0], 3), dtype=np.uint8)
        for idx, c in enumerate(colors):
            packed = struct.unpack('>l', struct.pack('>f', c))[0]
            unpacked_colors[idx, 0] = (packed >> 16) & 255  # Red
            unpacked_colors[idx, 1] = (packed >> 8) & 255   # Green
            unpacked_colors[idx, 2] = packed & 255          # Blue



        # Red filter points based on x < 1.0 and "red" color
        red_filter = (unpacked_colors[:, 0] > 200) & \
                    (unpacked_colors[:, 1] < 0.5 * unpacked_colors[:, 0]) & \
                    (unpacked_colors[:, 2] < 0.5 * unpacked_colors[:, 0])
        x_filter = points[:, 0] < 0.3 # x < 1.0
        red_combined_filter = red_filter & x_filter

        # Apply the filter
        red_filtered_points = points[red_combined_filter]
        red_filtered_colors = unpacked_colors[red_combined_filter]
        
        # Green filter points based on x < 1.0 and "green" color
        green_filter = (unpacked_colors[:, 1] > 110) & \
                    (unpacked_colors[:, 0] < 0.5 * unpacked_colors[:, 1]) & \
                    (unpacked_colors[:, 2] <  1*unpacked_colors[:, 1])
        """ green_filter = (unpacked_colors[:, 1] > 110) & \
                    (unpacked_colors[:, 0] < 0.5 * unpacked_colors[:, 1]) & \
                    (unpacked_colors[:, 2] <  1*unpacked_colors[:, 1]) """

        x_filter = points[:, 0] < 0.3  # x < 1.0
        green_combined_filter = green_filter & x_filter

        # Apply the filter
        green_filtered_points = points[green_combined_filter]
        green_filtered_colors = unpacked_colors[green_combined_filter]

        header=msg.header
        
        self.filtered_point_finder( green_filtered_points,green_filtered_colors,'g',header)
        self.filtered_point_finder( red_filtered_points,red_filtered_colors,'r',header)
        


    def filtered_point_finder( self,filtered_points,filtered_colors, colour , header):
        
        if filtered_points.shape[0] > 30:

            # Calculate the center of the points (simple centroid method
            center = np.mean(filtered_points, axis=0)

            # Calculate the mean distance from each point to the centroid
            distances = np.linalg.norm(filtered_points - center, axis=1)
            mean_distance = np.mean(distances)

            dist_filter = distances<0.10
            # Apply the filter
            filtered_points = filtered_points[dist_filter]
            filtered_colors = filtered_colors[dist_filter]
            # Calculate the center of the points (simple centroid method
            center = np.mean(filtered_points, axis=0)


            if mean_distance<0.10:

                # Pack the filtered colors back into float for ROS PointCloud2
                packed_colors = []
                for color in filtered_colors:
                    packed_color = (color[0] << 16) | (color[1] << 8) | color[2]
                    packed_colors.append(struct.unpack('>f', struct.pack('>l', packed_color))[0])
                packed_colors = np.array(packed_colors)

                # Combine filtered points and packed colors
                filtered_data = np.column_stack((filtered_points, packed_colors))

                # Define PointCloud2 fields for XYZ and RGB
                fields = [
                    pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name='rgb', offset=12, datatype=pc2.PointField.FLOAT32, count=1),
                ]

                # Convert NumPy array -> ROS PointCloud2
                
                filtered_msg = pc2.create_cloud(header=header, fields=fields, points=filtered_data)

                # Publish the filtered point cloud
                self._pub.publish(filtered_msg)
                if colour=='r':
                    self.get_logger().info(f"\033[31mFound red ball cloud with {filtered_points.shape[0]} points.\033[0m")
                    # Publish the transform to the sphere's center
                    self.publish_transform(header, center,'red_sphere')
                else:
                    self.get_logger().info(f"\033[32mFound green square cloud with {filtered_points.shape[0]} points.\033[0m")
                    # Publish the transform to the sphere's center
                    self.publish_transform(header, center,'green_square')
                return
        else:
            #self.get_logger().info(f"Nothing")
            return

    def publish_transform(self,header, center,name):
        """Publish a transform from 'map' to the detected sphere."""

        # Preserved but commented: Define frames and attempt asynchronous transform lookup to reference detected object
        to_frame_rel = 'map'
        from_frame_rel = header.frame_id
        time = header.stamp

        # # Wait for the transform asynchronously
        transform_future = self.tf_buffer.wait_for_transform_async(
            target_frame=to_frame_rel,
            source_frame=from_frame_rel,
            time=time
        )

        # # Spin until the future is complete or a timeout occurs
        rclpy.spin_until_future_complete(self, transform_future, timeout_sec=0.5)

        # Check if the future completed successfully
        if not transform_future.done():
            self.get_logger().error(
                f"Transform future did not complete successfully for Ball")
            return

        try:
            # Preserved but commented: Retrieve and use the transform
            transform = transform_future.result()
            pose = tf2_geometry_msgs.Pose()
            
            #pose.position = tf2_geometry_msgs.Point(x=center[2]+ self.camera_offset_z , y=-center[1]+ self.camera_offset_y, z=center[0]+ self.camera_offset_x)
            pose.position = tf2_geometry_msgs.Point(x=center[0] , y=center[1], z=center[2])



            trans_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)    
            
            t = TransformStamped()
            t.header.stamp = header.stamp  # Use the same timestamp as the original message
            t.header.frame_id = 'map'
            t.child_frame_id = name

            t.transform.translation.x = trans_pose.position.x
            t.transform.translation.y = trans_pose.position.y
            t.transform.translation.z = trans_pose.position.z            

            # No rotation for simplicity (identity quaternion)
            t.transform.rotation.x = 0.0#trans_pose.orientation.x
            t.transform.rotation.y = 0.0#trans_pose.orientation.y
            t.transform.rotation.z = 0.0#trans_pose.orientation.z
            t.transform.rotation.w = 1.0#trans_pose.orientation.w

            # Broadcast the transform
            self._tf_broadcaster.sendTransform(t)
            return
        except Exception as ex:
                # Log any errors (this will only log broadcasting issues now)
                self.get_logger().error(
                    f"Failed to process ball Error: {ex}")
                return

def main():
    rclpy.init()
    node = Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
