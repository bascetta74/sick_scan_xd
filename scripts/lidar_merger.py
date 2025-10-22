import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from laser_geometry.laser_geometry import LaserProjection

class LidarMerger(Node):
    def __init__(self):
        super().__init__('lidar_merger')
        
        # Subscribers for both PointCloud2 and LaserScan
        self.sub_pc1 = self.create_subscription(
            PointCloud2, '/cloud_1', self.callback_lidar1, 50)
        self.sub_pc2 = self.create_subscription(
            PointCloud2, '/cloud_2', self.callback_lidar2, 50)
        
        # Additional subscribers for LaserScan if available
        self.sub_scan1 = self.create_subscription(
            LaserScan, '/scan_1', self.callback_scan1, 50)
        self.sub_scan2 = self.create_subscription(
            LaserScan, '/scan_2', self.callback_scan2, 50)
        
        # Publishers
        self.pub_pc = self.create_publisher(PointCloud2, 'merged_pointcloud', 10)
        self.pub_scan = self.create_publisher(LaserScan, 'merged_laserscan', 10)
        
        self.lp = LaserProjection()
        self.cloud1 = None
        self.cloud2 = None
        self.scan1 = None
        self.scan2 = None
        
        # Transformation parameters
        self.translation = np.array([-0.90, 0.32, 0])  # (x, y, z)

        self.pitch = 0  # Rotation around X-axis
        self.yaw = 180   # Rotation around Z-axis
        self.roll = 0   # Rotation around Y-axis

        # Convert Euler angles from degrees to radians
        self.pitch = np.deg2rad(self.pitch)
        self.yaw = np.deg2rad(self.yaw)
        self.roll = np.deg2rad(self.roll)

        # Compute the rotation matrix
        self.R = self.euler_to_rotation_matrix(self.pitch, self.yaw, self.roll)        
        
        # For scan merging
        self.scan_angle_min = -np.pi
        self.scan_angle_max = np.pi
        self.scan_angle_increment = np.deg2rad(0.33)  
        self.range_min = 0.1
        self.range_max = 10.0

    def callback_lidar1(self, msg):
        self.cloud1 = msg
        self.merge_pointclouds()
    
    def callback_lidar2(self, msg):
        self.cloud2 = msg
        self.merge_pointclouds()
        
    def callback_scan1(self, msg):
        self.scan1 = msg
        self.merge_scans()
    
    def callback_scan2(self, msg):
        self.scan2 = msg
        self.merge_scans()
    
    def merge_pointclouds(self):
        if self.cloud1 is None or self.cloud2 is None:
            return

          # Read points from cloud1 and cloud2
        raw_points1 = list(pc2.read_points(self.cloud1, field_names=['x', 'y', 'z'], skip_nans=True))
        raw_points2 = list(pc2.read_points(self.cloud2, field_names=['x', 'y', 'z'], skip_nans=True))

        if len(raw_points1) == 0 or len(raw_points2) == 0:      
            self.get_logger().warn("One or both PointCloud2 messages are empty")
            return

        # Convert to numpy arrays
        points1 = np.array([(p[0], p[1], p[2]) for p in raw_points1], dtype=np.float32)
        points2 = np.array([(p[0], p[1], p[2]) for p in raw_points2], dtype=np.float32)

        # Apply the rotation and translation to cloud2 points
        points2_transformed = np.dot(self.R, points2.T).T + self.translation  # Apply rotation and translation

        # Debugging: Print a few transformed points to verify the result
        #self.get_logger().info(f"Transformed first 5 points of cloud2: {points2_transformed[:5]}")

        # Merge the transformed points of cloud2 with cloud1
        merged_points = np.vstack((points1, points2_transformed))

        # Debugging: Print number of points after merge
        #self.get_logger().info(f"Merged point cloud size: {len(merged_points)}")

        # Create a new PointCloud2 message with the merged points
        merged_cloud = pc2.create_cloud_xyz32(self.cloud1.header, merged_points)
        self.pub_pc.publish(merged_cloud)
    
    # def merge_scans(self):
    #     if self.scan1 is None or self.scan2 is None:
    #         return

    #     # Create output scan with parameters matching the input scans
    #     merged_scan = LaserScan()
    #     merged_scan.header = self.scan1.header
    #     merged_scan.angle_min = self.scan_angle_min
    #     merged_scan.angle_max = self.scan_angle_max
    #     merged_scan.angle_increment = self.scan_angle_increment
    #     merged_scan.time_increment = 0.0
    #     merged_scan.scan_time = 0.1
    #     merged_scan.range_min = min(self.scan1.range_min, self.scan2.range_min)
    #     merged_scan.range_max = max(self.scan1.range_max, self.scan2.range_max)
        
    #     # Calculate number of beams
    #     num_beams = int(round((merged_scan.angle_max - merged_scan.angle_min) / merged_scan.angle_increment))
    #     merged_scan.ranges = [float('inf')] * num_beams
        
    #     # Merge scan1 (no transformation needed)
    #     self.merge_single_scan(self.scan1, merged_scan)
        
    #     # Merge scan2 (with 180° yaw transformation)
    #     self.merge_single_scan(self.scan2, merged_scan, transform=True)
        
    #     # Convert inf to 0 for no detection
    #     merged_scan.ranges = [r if r != float('inf') else 0.0 for r in merged_scan.ranges]
        
    #     self.pub_scan.publish(merged_scan)

    def merge_scans(self):
        if self.scan1 is None or self.scan2 is None:
            return

        # Create output scan with correct timestamp
        merged_scan = LaserScan()
        merged_scan.header.frame_id = self.scan1.header.frame_id
        merged_scan.header.stamp = self.get_clock().now().to_msg()
        
        merged_scan.angle_min = self.scan_angle_min
        merged_scan.angle_max = self.scan_angle_max
        merged_scan.angle_increment = self.scan_angle_increment
        merged_scan.time_increment = 0.0
        merged_scan.scan_time = 0.1
        merged_scan.range_min = min(self.scan1.range_min, self.scan2.range_min)
        merged_scan.range_max = max(self.scan1.range_max, self.scan2.range_max)

        num_beams = int(round((merged_scan.angle_max - merged_scan.angle_min) / merged_scan.angle_increment))
        merged_scan.ranges = [float('inf')] * num_beams

        self.merge_single_scan(self.scan1, merged_scan)
        self.merge_single_scan(self.scan2, merged_scan, transform=True)
        
        merged_scan.ranges = [r if r != float('inf') else 0.0 for r in merged_scan.ranges]

        self.pub_scan.publish(merged_scan)


    def merge_single_scan(self, scan, merged_scan, transform=False):
        for i, distance in enumerate(scan.ranges):
            # Skip invalid measurements
            if distance < scan.range_min or distance > scan.range_max:
                continue
                
            # Calculate original angle
            angle = scan.angle_min + i * scan.angle_increment
            
            if transform:
                # Apply 180° rotation (just flip the angle)
                angle = angle + np.pi
                # Normalize angle to [-π, π]
                angle = np.arctan2(np.sin(angle), np.cos(angle))
                
                # Apply translation (convert to cartesian, translate, then back to polar)
                x = distance * np.cos(angle) + self.translation[0]
                y = distance * np.sin(angle) + self.translation[1]
                distance = np.sqrt(x**2 + y**2)
                angle = np.arctan2(y, x)
            
            # Find the appropriate bin in the merged scan
            beam_idx = int(round((angle - merged_scan.angle_min) / merged_scan.angle_increment))
            
            # Keep the closest point in each angular bin
            if 0 <= beam_idx < len(merged_scan.ranges):
                if distance < merged_scan.ranges[beam_idx]:
                    merged_scan.ranges[beam_idx] = distance

    def euler_to_rotation_matrix(self, pitch, yaw, roll):
        """Convert Euler angles to a rotation matrix."""
        # Rotation matrix for pitch (around x-axis)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch), np.cos(pitch)]
        ])
        
        # Rotation matrix for yaw (around z-axis)
        R_y = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Rotation matrix for roll (around y-axis)
        R_z = np.array([
            [np.cos(roll), 0, np.sin(roll)],
            [0, 1, 0],
            [-np.sin(roll), 0, np.cos(roll)]
        ])
        
        # Final rotation matrix by multiplying yaw, pitch, and roll
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R
    
def main(args=None):
    rclpy.init(args=args)
    node = LidarMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()