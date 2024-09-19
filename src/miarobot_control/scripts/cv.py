#!/usr/bin/env python3

import rospy
import cv2
import numpy as npz
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class KinectYOLOBallDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.color_image = None

        # Load the pre-trained YOLO model
        self.model = YOLO(r"D:\college\teams\test\best.pt")
        
        # ROS publishers for distance and angle
        self.distance_pub = rospy.Publisher('/ball/distance', Float32, queue_size=10)
        self.angle_pub = rospy.Publisher('/ball/angle', Float32, queue_size=10)
        
        # Subscribe to Kinect depth and color image topics
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.color_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.color_callback)

    def depth_callback(self, data):
        try:
            # Convert the ROS depth image to an OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)

    def color_callback(self, data):
        try:
            # Convert the ROS RGB image to an OpenCV image
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.detect_ball()
        except CvBridgeError as e:
            rospy.logerr(e)

    def detect_ball(self):
        if self.color_image is None or self.depth_image is None:
            return

        # Use YOLO to detect objects in the image
        results = self.model.predict(source=self.color_image)
        for result in results:
            for box in result.boxes:
                # Get the class ID and coordinates of the detected object
                class_id = int(box.cls[0])
                if class_id == 0:  # Assuming 0 is the class ID for the ball
                    # Get the bounding box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # Calculate the center of the bounding box
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    # Get the depth value at the center of the bounding box
                    distance = self.depth_image[center_y, center_x]

                    # Calculate the angle to the ball
                    angle = self.calculate_angle(center_x, center_y)

                    # Publish the distance and angle to ROS topics
                    self.publish_distance_and_angle(distance, angle)

                    # Draw the bounding box and center point on the image
                    cv2.rectangle(self.color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(self.color_image, (center_x, center_y), 5, (0, 0, 255), -1)

                    # Display the image
                    resized_img = cv2.resize(self.color_image, (800, 600))
                    cv2.imshow("Ball Detection", resized_img)
                    cv2.waitKey(1)

    def publish_distance_and_angle(self, distance, angle):
        # Publish distance and angle to their respective topics
        self.distance_pub.publish(Float32(distance))
        self.angle_pub.publish(Float32(angle))
        rospy.loginfo(f"Published Ball Distance: {distance} mm")
        rospy.loginfo(f"Published Ball Angle: {angle:.2f} degrees")

    def calculate_angle(self, x, y):
        """
        Calculate the angle of the ball relative to the center of the image (optical axis).
        """
        # Image center
        image_center_x = self.color_image.shape[1] / 2

        # Horizontal field of view (in degrees) for Kinect v1: 57 degrees
        hfov = 57.0

        # Calculate the angle relative to the center of the image
        delta_x = x - image_center_x

        # Convert pixel difference to angle
        angle = (delta_x / image_center_x) * (hfov / 2)
        return angle

def main():
    rospy.init_node('kinect_yolo_ball_detector', anonymous=True)
    detector = KinectYOLOBallDetector()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()