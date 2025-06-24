#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
from sensor_msgs_py import point_cloud2

class PointCloudMerger(Node):
    def __init__(self):
        super().__init__('point_cloud_merger')
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers with correct topic names
        self.front_sub = Subscriber(self, PointCloud2, '/camera_front/camera_front/points')
        self.left_sub = Subscriber(self, PointCloud2, '/camera_left/camera_left/points')
        self.right_sub = Subscriber(self, PointCloud2, '/camera_right/camera_right/points')

        # Synchronizer
        self.ts = ApproximateTimeSynchronizer(
            [self.front_sub, self.left_sub, self.right_sub],
            queue_size=10,
            slop=0.5
        )
        self.ts.registerCallback(self.merge_callback)

        # Publisher
        self.merged_pub = self.create_publisher(PointCloud2, '/merged_points', 10)
        self.get_logger().info("Node initialized")

    def merge_callback(self, front_msg, left_msg, right_msg):
        try:
            # Get transforms
            now = rclpy.time.Time()
            tf_front = self.tf_buffer.lookup_transform('base_link', front_msg.header.frame_id, now)
            tf_left = self.tf_buffer.lookup_transform('base_link', left_msg.header.frame_id, now)
            tf_right = self.tf_buffer.lookup_transform('base_link', right_msg.header.frame_id, now)
    
            # Transform clouds
            front_trans = do_transform_cloud(front_msg, tf_front)
            left_trans = do_transform_cloud(left_msg, tf_left)
            right_trans = do_transform_cloud(right_msg, tf_right)
    
            # Extract points as numpy arrays (FIXED SYNTAX)
            front_pts = np.array(list(point_cloud2.read_points(front_trans, field_names=('x', 'y', 'z'), skip_nans=True)))
            left_pts = np.array(list(point_cloud2.read_points(left_trans, field_names=('x', 'y', 'z'), skip_nans=True)))
            right_pts = np.array(list(point_cloud2.read_points(right_trans, field_names=('x', 'y', 'z'), skip_nans=True)))
    
            # Merge points
            merged_pts = np.vstack([front_pts, left_pts, right_pts])
    
            # Create output message
            header = front_trans.header
            header.frame_id = 'base_link'
            
            # Create cloud using create_cloud_xyz32
            merged_cloud = point_cloud2.create_cloud_xyz32(header, merged_pts.tolist())
            
            self.merged_pub.publish(merged_cloud)
            self.get_logger().info(f"Published merged cloud with {len(merged_pts)} points", throttle_duration_sec=1)
    
        except Exception as e:
            self.get_logger().error(f"Merge error: {str(e)}", throttle_duration_sec=1)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()