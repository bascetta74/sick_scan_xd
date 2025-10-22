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
            '/cloud_1',  # Ensure correct topic name
            self.listener_callback_1,
            1  # Reduced queue size
        )

        # Subscribe to the second PointCloud2 topic
        self.subscription2 = self.create_subscription(
            PointCloud2,
            '/cloud_2',  # Ensure correct topic name
            self.listener_callback_2,
            1  # Reduced queue size
        )

        # Timer to check topic activity periodically
        self.timer = self.create_timer(2.0, self.check_topic_activity)
        self.received_msg1 = False
        self.received_msg2 = False

        self.get_logger().info("MultiPointCloudSaver node initialized.")

    def convert_and_save(self, msg, filename):
        """ Convert PointCloud2 message to PCD file using Open3D """
        # Extract points from PointCloud2 message (x, y, z fields)
        pc_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        if len(pc_data) == 0:
            self.get_logger().warn(f"Received empty point cloud for {filename}, skipping save.")
            return

        # Extract the x, y, z values directly from the structured array
        points = np.array([list(point) for point in pc_data], dtype=np.float32)

        # Create Open3D PointCloud object and assign points
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(points)

        # Save the point cloud to a PCD file
        o3d.io.write_point_cloud(filename, point_cloud)
        self.get_logger().info(f"Saved point cloud to {filename}")

    def listener_callback_1(self, msg):
        """ Callback for /cloud_1 topic """
        self.get_logger().info(f"Received PointCloud2 from topic /cloud_1 with {len(msg.data)} bytes")
        self.received_msg1 = True
        self.convert_and_save(msg, "output_topic1.pcd")

    def listener_callback_2(self, msg):
        """ Callback for /cloud_2 topic """
        self.get_logger().info(f"Received PointCloud2 from topic /cloud_2 with {len(msg.data)} bytes")
        self.received_msg2 = True
        self.convert_and_save(msg, "output_topic2.pcd")

    def check_topic_activity(self):
        """ Check if messages are being received from both topics """
        if not self.received_msg1:
            self.get_logger().warn("No messages received from /cloud_1 yet.")
        if not self.received_msg2:
            self.get_logger().warn("No messages received from /cloud_2 yet.")


def main(args=None):
    rclpy.init(args=args)
    node = MultiPointCloudSaver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

