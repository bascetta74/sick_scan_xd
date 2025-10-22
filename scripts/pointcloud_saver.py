import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d

class MultiPointCloudSaver(Node):
    def __init__(self):
        super().__init__('multi_pointcloud_saver')

        # Subscribe to the first PointCloud2 topic
        self.subscription1 = self.create_subscription(
            PointCloud2,
            '/lidar1/cloud1',  # Replace with your actual topic name
            self.listener_callback_1,
            10
        )

        # Subscribe to the second PointCloud2 topic
        self.subscription2 = self.create_subscription(
            PointCloud2,
            '/lidar2/cloud2',  # Replace with your actual topic name
            self.listener_callback_2,
            10
        )

        self.subscription1
        self.subscription2

    def convert_and_save(self, msg, filename):
        """ Convert PointCloud2 message to PCD file using Open3D """
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

        if points.size == 0:
            self.get_logger().warn(f"Received empty point cloud for {filename}, skipping save.")
            return

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)

        # Save the point cloud to a PCD file
        o3d.io.write_point_cloud(filename, point_cloud)
        self.get_logger().info(f"Saved point cloud to {filename}")

    def listener_callback_1(self, msg):
        self.get_logger().info("Received PointCloud2 from topic 1")
        self.convert_and_save(msg, "output_topic1.pcd")

    def listener_callback_2(self, msg):
        self.get_logger().info("Received PointCloud2 from topic 2")
        self.convert_and_save(msg, "output_topic2.pcd")

def main(args=None):
    rclpy.init(args=args)
    node = MultiPointCloudSaver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

