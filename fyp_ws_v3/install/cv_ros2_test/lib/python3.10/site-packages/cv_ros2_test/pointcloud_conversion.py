import rclpy
import numpy as np

from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from tf2_geometry_msgs import PoseStamped
from std_msgs.msg import Int32MultiArray

cam_x_pos = None
cam_y_pos = None
rz = None
from_frame_rel = 'zed_left_camera_frame'

def euler_to_quaternion(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qw, qx, qy, qz]

class Listener(Node):
    def __init__(self):
        super().__init__('pointcloud_node')        
        self.subscription = self.create_subscription(Int32MultiArray, '/opencv_pose', self.opencv_callback, 10)
        self.subscription = self.create_subscription(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', self.pointcloud_callback, 10)
        self.publisher_ = self.create_publisher(PoseStamped,'obj_pose',10)

    def opencv_callback(self,msg):
        global cam_x_pos
        global cam_y_pos
        global rz
        data = msg.data
        cam_x_pos = data[0]
        cam_y_pos = data[1]
        rz = data[2]


    def pointcloud_callback(self,data):
        global base_pose
        now = rclpy.time.Time()

        if (cam_x_pos != 0) and (cam_y_pos != 0):
            pointcloud_height = 360
            pointcloud_width = 640
            pointcloud_index = cam_x_pos + (cam_y_pos*pointcloud_width)
            points = pc2.read_points_list(data,field_names=("x", "y", "z"), skip_nans=False)
            x, y, z = points[pointcloud_index]
            float_x = float(x)
            float_y = float(y) - 0.3
            float_z = float(z)

            q = euler_to_quaternion(3.132, 0.0, (-2.168 + rz))

            base_pose = PoseStamped()
            base_pose.header.frame_id = from_frame_rel
            base_pose.header.stamp = now.to_msg()
            base_pose.pose.position.x = float_x
            base_pose.pose.position.y = float_y
            base_pose.pose.position.z = float_z
            base_pose.pose.orientation.w = q[0]
            base_pose.pose.orientation.x = q[1]
            base_pose.pose.orientation.y = q[2]
            base_pose.pose.orientation.z = q[3]
            self.publisher_.publish(base_pose)

def main(args=None):
    rclpy.init(args=args)
    publisher = Listener()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()