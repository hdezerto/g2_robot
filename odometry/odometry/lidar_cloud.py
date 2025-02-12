
import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class LidarHybridVisualizer(Node):
    def __init__(self, nth_scan=5):
        super().__init__('lidar_hybrid_visualizer')

        self.publisher = self.create_publisher(PointCloud2, '/map', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.listener_callback, 10)
        
        self.proj = LaserProjection()
        
        self.nth_scan = nth_scan  # Publish every Nth scan
        self.scan_count = 0

        # Hybrid storage: keep both raw scans & transformed scans
        self.raw_scans = []       # Stores original scans
        self.nth_transformed_scans = []  # Stores every Nth transformed scan

    def listener_callback(self, msg):
        self.scan_count += 1

        # Only process every Nth scan
        if self.scan_count % self.nth_scan != 0:
            return  

        # Convert LaserScan to PointCloud2 (assuming the scan is already in map frame)
        cloud = self.proj.projectLaser(msg)

        # Hybrid storage: Save raw scan
        self.raw_scans.append(cloud)

        # Store every Nth scan for later publishing
        self.nth_transformed_scans.append(cloud)

        # Publish transformed scans (for visualizing drift)
        for scan in self.raw_scans:
            self.publisher.publish(scan)

        # Aggregate and publish every Nth transformed scan as a PointCloud2
        aggregated_points = []
        for scan in self.nth_transformed_scans:
            # Extracting points from the PointCloud2 message
            pc_points = pc2.read_points(scan, field_names=("x", "y", "z"), skip_nans=True)
            for point in pc_points:
                aggregated_points.append([point[0], point[1], point[2]])

        # Convert aggregated points to a NumPy array
        aggregated_points_array = np.array(aggregated_points, dtype=np.float32)

        # Create a new PointCloud2 message for every Nth scan
        header = msg.header
        point_cloud_msg = pc2.create_cloud_xyz32(header, aggregated_points_array)

        # Publish the aggregated PointCloud2
        self.publisher.publish(point_cloud_msg)

        self.get_logger().info(f'Published {len(self.nth_transformed_scans)} Nth transformed scans showing drift')

    def reprocess_with_updated_transforms(self):
        """ Reapply updated transforms to raw scans if needed (for correction) """
        self.transformed_scans.clear()
        
        for raw_scan in self.raw_scans:
            # Since no transform is needed, we can just append the raw scan
            self.transformed_scans.append(raw_scan)

        self.get_logger().info(f'Reprocessed {len(self.transformed_scans)} scans.')

def main():
    rclpy.init()
    node = LidarHybridVisualizer(nth_scan=5)  # Change 5 to adjust scan frequency
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
