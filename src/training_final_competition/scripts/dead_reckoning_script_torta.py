#!/usr/bin/env python3

from dead_reckoning import ForwardKinematics, Mecanum
import rospy
import dead_reckoning
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Float32, Bool
from math import pi
from tf.transformations import euler_from_quaternion
import math
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf

Odom_set = Odometry

rate_value = 50
lx = 0.175
ly = 0.075
wheel_radius = 0.04
speed_set = False
odom_restart_flag =False

def RPM_to_rad_per_sec(speeds):
    return 2 *pi * speeds /60

def speeds_of_wheels_callback(data: Float32MultiArray):
    global speeds_of_wheels,speed_set
    speeds_of_wheels = np.array(data.data)
    speeds_of_wheels = RPM_to_rad_per_sec(speeds_of_wheels)
    speed_set = True

def theta_callback (imuMsg:Float32):
    
    global theta
    x, y, z, w = imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w
    roll, pitch, yaw  = euler_from_quaternion([x, y, z, w])
    theta = yaw

def odom_set_callback (data:Odometry):
    Odom_set.pose.pose.position.x    = data.pose.pose.position.x
    Odom_set.pose.pose.position.y    = data.pose.pose.position.y
    Odom_set.pose.pose.orientation.w = data.pose.pose.orientation.w
    Odom_set.pose.pose.orientation.x = data.pose.pose.orientation.x
    Odom_set.pose.pose.orientation.y = data.pose.pose.orientation.y
    Odom_set.pose.pose.orientation.z = data.pose.pose.orientation.z

def odom_set_flag_callback (data:Bool):
    odom_restart_flag = data.data

if __name__ == "__main__" :
    speeds_of_wheels = []
    theta = 0

    mecanum = Mecanum(lx , ly , wheel_radius)
    odom_broadcaster = tf.TransformBroadcaster()
    
    rospy.init_node("dead_reckoning")
    rospy.Subscriber("/Espeeds", Float32MultiArray, speeds_of_wheels_callback)
    rospy.Subscriber("/odom_set_flag", Bool, odom_set_flag_callback)
    rospy.Subscriber("/odom_set", Odometry, odom_set_callback)

    rospy.Subscriber("/imu", Float32, theta_callback)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    rate = rospy.Rate(rate_value)

    while not rospy.is_shutdown():
        if not speed_set:
            continue
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        base_link_speeds = mecanum.calc_speed(speeds_of_wheels)
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        integrated_distances,odom_speeds  = mecanum.calc_distance(dt, base_link_speeds,theta)

        x, y = integrated_distances[0], integrated_distances[1]
        vx, vy, vth = odom_speeds[0],odom_speeds[1],odom_speeds[2]

        odom_broadcaster.sendTransform(
        (integrated_distances[0], integrated_distances[1], 0),
        odom_quat,
        current_time,
        "base_link",
        "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        if odom_restart_flag == True :
            odom.pose.pose.position.x    = Odom_set.pose.pose.position.x
            odom.pose.pose.position.y    = Odom_set.pose.pose.position.y
            odom.pose.pose.orientation.w = Odom_set.pose.pose.orientation.w 
            odom.pose.pose.orientation.x = Odom_set.pose.pose.orientation.x
            odom.pose.pose.orientation.y = Odom_set.pose.pose.orientation.y 
            odom.pose.pose.orientation.z = Odom_set.pose.pose.orientation.z
        else:
            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)
        last_time = current_time
        rate.sleep()

 