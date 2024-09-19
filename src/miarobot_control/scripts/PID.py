#! usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, Pose2D, Point, Twist, Vector3
from nav_msgs.msg import Odometry
import math

MAX_SPEED = 1

def normalize_angle(angle):
    angle = math.fmod(angle + math.pi, 2.0 * math.pi)
    if angle < 0:
        angle += 2.0 * math.pi
    angle -= math.pi
    return angle

class AxisRot:
    def __init__(self, yaw=0, pitch=0, roll=0):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

    def from_quaternion(self, q):
        tf2_matrix = tf2_ros.transform_to_matrix(tf2_geometry_msgs.from_msg(q))
        roll, pitch, yaw = tf2_ros.euler_from_matrix(tf2_matrix)
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll

class FetchData:
    def __init__(self):
        self.set_x = 0
        self.set_y = 0
        self.set_theta = 0
        self.q = tf2_geometry_msgs.Quaternion()
        self.p = Point()
        self.setPoint = Point()
        self.linear = Vector3()
        self.angular = Vector3()
        self.pose = Pose()

    def fetch_data(self, msg):
        self.q = tf2_geometry_msgs.from_msg(msg.pose.pose.orientation)
        self.p = tf2_geometry_msgs.from_msg(msg.pose.pose.position)
        self.linear = tf2_geometry_msgs.from_msg(msg.twist.twist.linear)
        self.angular = tf2_geometry_msgs.from_msg(msg.twist.twist.angular)
        self.pose = msg.pose.pose

    def fetch_point(self, msg):
        self.setPoint = tf2_geometry_msgs.from_msg(msg)
        self.set_theta = normalize_angle(msg.theta)

class Pid:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.accum = 0
        self.prev_err = 0
        self.prev_time = rospy.Time.now()

    def get_output(self, target, current):
        err = target - current
        dt = (rospy.Time.now() - self.prev_time).to_sec()

        p = self.kp * err
        d = self.kd * (self.prev_err - err) / dt
        self.accum += err * dt
        self.accum = min(max(self.accum, -MAX_SPEED), MAX_SPEED)

        i = self.ki * self.accum

        self.prev_err = err
        return p + d + i

def publish_output(msg, pub):
    pub.publish(msg)

def transform_to_robot_frame(set_point, robot_pose):
    robot_transform = tf2_ros.transform_to_matrix(tf2_geometry_msgs.from_msg(robot_pose))
    global_point_transform = tf2_ros.transform_to_matrix(tf2_geometry_msgs.from_msg(set_point))
    relative_transform = tf2_ros.transform_multiply(tf2_ros.transform_inverse(robot_transform), global_point_transform)
    relative_position = tf2_ros.transform_to_translation(relative_transform)
    transformed_point = tf2_geometry_msgs.from_transform(tf2_ros.transform_from_translation_rotation(relative_position, [0, 0, 0]))
    return transformed_point

def main():
    data = FetchData()
    x_pid = Pid(1, 0.01, 0.1)
    y_pid = Pid(1, 0.01, 0.1)
    theta_pid = Pid(1, 0.01, 0.1)

    rospy.init_node('PID')
    sub = rospy.Subscriber('/odom', 1000, data.fetch_data)
    sub2 = rospy.Subscriber('/setpoint', 100, data.fetch_point)
    pub = rospy.Publisher('/cmd_vel', Twist, 1000)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        robot_set_point = transform_to_robot_frame(data.setPoint, data.pose)
        set_x = robot_set_point.x
        set_y = robot_set_point.y

        rot_rpy = AxisRot()
        rot_rpy.from_quaternion(data.q)
        x_out = min(MAX_SPEED, x_pid.get_output(set_x, data.linear.x))
        y_out = min(MAX_SPEED, y_pid.get_output(set_y, data.linear.y))
        theta_out = min(MAX_SPEED, theta_pid.get_output(data.set_theta, rot_rpy.yaw))

        msg = Twist()
        msg.angular.z = theta_out
        msg.linear.x = x_out
        msg.linear.y = y_out

        publish_output(msg, pub)

        rospy.spinOnce()
        rate.sleep()

if __name__ == '__main__':
    main()