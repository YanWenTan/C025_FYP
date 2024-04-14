import rclpy

from rclpy.node import Node
import rclpy.time

import tf2_ros as tf2
from tf2_ros import TransformException
from tf2_geometry_msgs import PoseStamped

base_pose = PoseStamped()
transformed_pose = PoseStamped()
from_frame_rel = 'zed_left_camera_frame'
to_frame_rel = 'world'

class Listener(Node):
    def __init__(self):
        super().__init__('transform_node')

        self.tf_buffer = tf2.Buffer()
        self.tf_listener = tf2.TransformListener(self.tf_buffer, self)
        self.tf_buffer_interface = tf2.BufferInterface()
        tf_future = self.tf_buffer.wait_for_transform_async(to_frame_rel, from_frame_rel, rclpy.time.Time())
        rclpy.spin_until_future_complete(self, tf_future)
        self.subscription = self.create_subscription(PoseStamped,'obj_pose', self.transform_callback, 1)
        self.publisher_ = self.create_publisher(PoseStamped,'transformed_pose',1)

    def transform_callback(self, data):
        now = rclpy.time.Time()
        base_pose = data

        base_pose.header.frame_id = from_frame_rel
        base_pose.header.stamp = now.to_msg()
        base_pose.pose.position.x = base_pose.pose.position.x
        base_pose.pose.position.y = base_pose.pose.position.y
        base_pose.pose.position.z = base_pose.pose.position.z

        trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time(), rclpy.duration.Duration(seconds = 0.1))
        transformed_pose = self.tf_buffer.transform(base_pose, to_frame_rel, rclpy.duration.Duration(seconds = 0.1))
        transformed_pose.pose.orientation.w = base_pose.pose.orientation.w
        transformed_pose.pose.orientation.x = base_pose.pose.orientation.x
        transformed_pose.pose.orientation.y = base_pose.pose.orientation.y
        transformed_pose.pose.orientation.z = base_pose.pose.orientation.z
        self.publisher_.publish(transformed_pose)

def main(args=None):
    rclpy.init(args=args)
    publisher = Listener()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
