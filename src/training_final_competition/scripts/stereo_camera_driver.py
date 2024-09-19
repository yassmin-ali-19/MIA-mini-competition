#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
import subprocess
import glob
from typing import List
from cv_bridge import CvBridge
import yaml
import rospkg

class Camera:
    def __init__(self, id: int, name: str) -> None:
        self.id = id
        self.name = name

class StereoCameraStreamer:
    def __init__(self, raw_topic="/stereo/rgb/image_raw", left_topic="/camera/rgb/image_raw", right_topic="/stereo/right/image_raw",
                 left_calibration_topic="/stereo/left/camera_info", right_calibration_topic="/stereo/right/camera_info",
                 calibration_left="/calibration/left.yaml",
                 calibration_right="/calibration/right.yaml",
                 camera_name="3D USB Camera", auto_detect_camera=True, split_image=True,
                 camera_id=4, frame_width=1280, frame_height=480,
                 publish_raw=True, publish_left_right=True):
        self.raw_topic = raw_topic
        self.left_topic = left_topic
        self.right_topic = right_topic
        self.left_calibration_topic = left_calibration_topic
        self.right_calibration_topic = right_calibration_topic
        self.calibration_left = calibration_left
        self.calibration_right = calibration_right
        self.camera_name = camera_name
        self.auto_detect_camera = auto_detect_camera
        self.split_image = split_image
        self.camera_id = camera_id
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.publish_raw = publish_raw
        self.publish_left_right = publish_left_right
        self.cap = None
        self.current_image = None
        self.current_left_image = None
        self.current_right_image = None
        if self.publish_raw is True:
            self.pub_raw = rospy.Publisher(self.raw_topic, Image, queue_size=10)

        if self.split_image:
            # self.cinfo_left = CameraInfoManager(cname="stereo_left", url=self.calibration_left, namespace="/left")
            # self.cinfo_left.loadCameraInfo()

            # self.cinfo_right = CameraInfoManager(cname="stereo_right", url=self.calibration_right, namespace="/right")
            # self.cinfo_right.loadCameraInfo()
            
            # self.cinfo_left = self.loadCameraInfo(self.calibration_left)
            # self.cinfo_right = self.loadCameraInfo(self.calibration_right)

            if self.publish_left_right is True:
                self.pub_left = rospy.Publisher(self.left_topic, Image, queue_size=10)
                # self.cinfo_left_pub = rospy.Publisher(self.left_calibration_topic, CameraInfo, queue_size=10)
                self.pub_right = rospy.Publisher(self.right_topic, Image, queue_size=10)
                # self.cinfo_right_pub = rospy.Publisher(self.right_calibration_topic, CameraInfo, queue_size=10)

        if self.auto_detect_camera:
            self.detect_camera()

        self.set_frame_size()

    def loadCameraInfo(self, yaml_fname):
        package_path = rospkg.RosPack().get_path('ball_selection')
        yaml_fname = package_path + yaml_fname
        
        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.safe_load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg

    def get_connected_cameras(self) -> List[Camera]:
        cameras = []

        cameras_paths = glob.glob("/dev/video?")
        for camera_path in cameras_paths:
            camera_index = int(camera_path.replace('/dev/video', ''))
            camera_name = subprocess.run(['cat', '/sys/class/video4linux/video{}/name'.format(camera_index)],
                                        stdout=subprocess.PIPE).stdout.decode('utf-8')
            camera_name = camera_name.replace('\n', '')
            cameras.append(Camera(camera_index, camera_name))

        return cameras

    def get_available_cameras(self) -> List[Camera]:
        available_cameras = []
        cameras = self.get_connected_cameras()
        for camera in cameras:
            self.capture = cv2.VideoCapture(camera.id)
            if self.capture.read()[0]:
                available_cameras.append(camera)
                self.capture.release()
        return available_cameras
    
    def detect_camera(self):
        # tests all connected cameras for valid connections
        cameras = self.get_available_cameras()
        for camera in cameras:
            if self.camera_name.lower() in camera.name.lower():
                self.camera_id = camera.id
                break

    def set_frame_size(self):
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        rospy.loginfo(f"Started streaming from camera {self.camera_id} at {actual_width}x{actual_height}")
    
    def stream_stereo_camera(self):
        ret, frame = self.cap.read()
        self.current_image = frame
        
        if frame is None:
            rospy.loginfo("No camera frame received")
            return

        raw_frame_msg = CvBridge().cv2_to_imgmsg(frame, encoding='bgr8')
        raw_frame_msg.header.frame_id = "camera_optical_frame"
        header = raw_frame_msg.header
        header.stamp = rospy.Time.now()
        self.pub_raw.publish(raw_frame_msg)
        if self.split_image:
            lr_boundary = int(frame.shape[1]/2)

            frameL = frame[:, :lr_boundary]
            frameR = frame[:, lr_boundary:]

            self.current_left_image = frameL
            self.current_right_image = frameR

            left_frame_msg = CvBridge().cv2_to_imgmsg(frameL, encoding='bgr8')
            left_frame_msg.header = header
            right_frame_msg = CvBridge().cv2_to_imgmsg(frameR, encoding='bgr8')
            right_frame_msg.header = header


            # self.cinfo_left_msg = self.cinfo_left
            # self.cinfo_right_msg = self.cinfo_right

            # self.cinfo_left_msg = self.cinfo_left.getCameraInfo()
            # self.cinfo_right_msg = self.cinfo_right.getCameraInfo()

            # self.cinfo_left_msg.header = header
            # self.cinfo_left_pub.publish(self.cinfo_left_msg)

            # self.cinfo_right_msg.header = header
            # self.cinfo_right_pub.publish(self.cinfo_right_msg)      

            self.pub_left.publish(left_frame_msg)
            self.pub_right.publish(right_frame_msg)
                
    def getCurrentImage(self):
        return self.current_image
    def getCurrentRGBRaw(self):
        return self.current_image
    def getCurrentLeftImage(self):
        return self.current_left_image
    
    def getCurrentImage(self):
        return self.current_right_image

if __name__ == "__main__":
    rospy.init_node('stereo_driver', anonymous=True)
    stereo_streamer = StereoCameraStreamer("/concatenated_img",camera_id=4)
    while not rospy.is_shutdown():
        stereo_streamer.stream_stereo_camera()
