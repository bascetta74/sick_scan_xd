import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import pcl
import sensor_msgs_py.point_cloud2 as pc2

class MultiPointCloudSaver(Node):
    def __init__(self):
        super().__init__('multi_pointcloud_saver')
        
        # Subscribe to the first PointCloud2 topic
        self.subscription1 = self.create_subscription(
            PointCloud2,
            'cloud_1',  # Replace with your first topic name
            self.listener_callback_1,
            10
        )
        
        # Subscribe to the second PointCloud2 topic
        self.subscription2 = self.create_subscription(
            PointCloud2,
            'cloud_2',  # Replace with your second topic name
            self.listener_callback_2,
            10
        )
        
        # Prevent unused variable warnings
        self.subscription1
        self.subscription2

    def listener_callback_1(self, msg):
        self.get_logger().info('Received PointCloud2 from topic 1')

        # Convert the PointCloud2 message to a list of points
        cloud_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Convert list of points to a pcl.PointCloud
        cloud = pcl.PointCloud()
        cloud.from_list(list(cloud_points))

        # Save the point cloud from topic 1 to a .pcd file
        cloud.to_file('output_topic1.pcd')

        self.get_logger().info('PointCloud from topic 1 saved as output_topic1.pcd')

    def listener_callback_2(self, msg):
        self.get_logger().info('Received PointCloud2 from topic 2')

        # Convert the PointCloud2 message to a list of points
        cloud_points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Convert list of points to a pcl.PointCloud
        cloud = pcl.PointCloud()
        cloud.from_list(list(cloud_points))

        # Save the point cloud from topic 2 to a .pcd file
        cloud.to_file('output_topic2.pcd')

        self.get_logger().info('PointCloud from topic 2 saved as output_topic2.pcd')


def main(args=None):
    rclpy.init(args=args)
    node = MultiPointCloudSaver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

