import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import os  

class MapBuilder:
    def __init__(self, resolution=0.1, size=500, folder="maps"):
        self.resolution = resolution  # 10 cm per pixel
        self.size = size  # pixels (map will be size x size)
        self.map = 205 * np.ones((size, size), dtype=np.uint8)  # Initialize as unknown (gray)
        self.folder = folder  
        os.makedirs(self.folder, exist_ok=True)

    def world_to_map(self, x, y):
        """ Convert real-world (meters) coordinates to map indices """
        mx = int(self.size / 2 + x / self.resolution)
        my = int(self.size / 2 - y / self.resolution)
        return mx, my

    def add_scan(self, scan_data):
        """ Process LiDAR scan data into the occupancy grid """
        for x, y in scan_data:
            mx, my = self.world_to_map(x, y)
            if 0 <= mx < self.size and 0 <= my < self.size:
                self.map[my, mx] = 50  # Dark gray near obstacles

    def add_object(self, x, y, label):
        """ Add objects with specific grayscale values based on label """
        mx, my = self.world_to_map(x, y)
        if 0 <= mx < self.size and 0 <= my < self.size:
            if label == 1:  # Square
                self.map[my, mx] = 100  # Medium gray
            elif label == 2:  # Ball
                self.map[my, mx] = 150  # Light gray
            elif label == 3:  # Fluffy animal
                self.map[my, mx] = 200  # Very light gray
            elif label == 'B':  # Box
                self.map[my, mx] = 0  # Black

    def save_map(self, filename="map.pgm"):
        """ Save map as a PGM file """
        filepath = os.path.join(self.folder, filename)
        cv2.imwrite(filepath, self.map)
        print(f"Map saved at {filepath}")

    def save_yaml(self, filename="map.yaml"):
        """ Save metadata as YAML file """
        filepath = os.path.join(self.folder, filename)
        yaml_content = f"""image: map.pgm
resolution: {self.resolution}
origin: [-{(self.size / 2) * self.resolution}, -{(self.size / 2) * self.resolution}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
"""
        with open(filepath, "w") as f:
            f.write(yaml_content)
        print(f"YAML metadata saved at {filepath}")

class LidarMapBuilder(Node):
    def __init__(self):
        super().__init__('lidar_map_builder')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.map_builder = MapBuilder()
        self.scan_count = 0  

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        valid_indices = np.isfinite(ranges)  
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        scan_data = np.vstack((x, y)).T

        self.map_builder.add_scan(scan_data)
        self.get_logger().info(f'Added scan to map. Total scans: {self.scan_count}')
        
        self.scan_count += 1  

        if self.scan_count > 100:  
            self.map_builder.save_map()
            self.map_builder.save_yaml()
            self.get_logger().info("Map and metadata saved in 'maps/' folder.")
            self.scan_count = 0  


def main():
    rclpy.init()
    node = LidarMapBuilder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
