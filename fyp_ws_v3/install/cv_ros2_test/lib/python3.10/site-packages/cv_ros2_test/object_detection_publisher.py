import rclpy
import numpy as np

from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2 as cv

from tf2_geometry_msgs import PoseStamped

rx = None
past_width = 0
past_height = 0
past_angle = 180
base_pose = PoseStamped()
image_offset_x = [100,250]
image_offset_y = [100,275]
from_frame_rel = 'zed_left_camera_frame'

def euler_to_quaternion(roll, pitch, yaw):
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qw, qx, qy, qz]

class Listener(Node):
    def __init__(self):
        super().__init__('opencv_node')
        self.br=CvBridge()
        
        self.subscription = self.create_subscription(Image, '/zed/zed_node/left/image_rect_color', self.image_callback, 1)
        self.subscription = self.create_subscription(PointCloud2, '/zed/zed_node/point_cloud/cloud_registered', self.pointcloud_callback, 10)
        self.publisher_ = self.create_publisher(PoseStamped,'obj_pose',10)


    def image_callback(self, image):
        global cam_x_pos
        global cam_y_pos
        global rz
        global past_width
        global past_height
        global past_angle
        cam_x_pos = None
        cam_y_pos = None
        cv_image = self.br.imgmsg_to_cv2(image)
        cropped_vid = cv_image[image_offset_y[0]:image_offset_y[1], image_offset_x[0]:image_offset_x[1]]
        grayscale = cv.cvtColor(cropped_vid, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(grayscale, 70, 255, 0)
        kernel = np.ones((5,5),np.uint8)
        opening = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
        contours, hierarchy = cv.findContours(opening, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for i in range(len(contours)):
            cnt = contours[i]
            rect = cv.minAreaRect(cnt)
            (x, y), (width, height), angle = rect
            if width>30 and height>30:
                if (x < 40):
                    break
                else:
                    box = cv.boxPoints(rect)
                    box = np.intp(box)
                    cv.drawContours(cropped_vid,[box],0,(0,0,255),2)
                    cam_x_pos = int(x) + image_offset_x[0]
                    cam_y_pos = int(y) + image_offset_y[0]
                    if (width<height):
                        if (angle>45):
                            angle = abs(angle-90)
                    elif (width>height):
                        if (angle>45):
                            angle = angle-90
                        else :
                            angle = -angle
                    rz = (angle) * (3.141/180)
                    self.get_logger().info(
                                f'Object Detected! \nAngle of Rotation:\t{rz} \nWidth:\t\t{width} \nHeight:\t\t{height}\n X Pos:\t\t{cam_x_pos}\n Y Pos:\t\t{cam_y_pos}')
                    past_width = 0
                    past_height = 0
                    past_angle = 180
                    break
        cv.imshow('gray', grayscale)
        cv.imshow('opening', opening)
        cv.imshow('image', cropped_vid)
        cv.waitKey(1)
        

    def pointcloud_callback(self,data):
        global base_pose
        now = rclpy.time.Time()

        if (cam_x_pos != None) and (cam_y_pos != None):
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