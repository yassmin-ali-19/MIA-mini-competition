#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D, Point
from nav_msgs.msg import Odometry
import math

current_point = Point()
pos_setpoints = []
neg_setpoints = []
setpoints = []
setpoint_index = 0
direction = 1 
tolerance = 0.05 

MOVING = "MOVING"
ARRIVED = "ARRIVED"
state = MOVING  

def generate_setpoints():
    max_y = 0.6
    step_x = 0.5
    step_y = 0.15
    start_x = 0.45

    for y in [i * step_y for i in range(int(max_y / step_y) + 1)]:
        pos_setpoints.append((start_x, y))
        pos_setpoints.append((start_x + step_x, y))
        pos_setpoints.append((start_x, y))
    
    for y in [-i * step_y for i in range(1, int(max_y / step_y) + 1)]:
        neg_setpoints.append((start_x, y))
        neg_setpoints.append((start_x + step_x, y))
        neg_setpoints.append((start_x, y))

    global setpoints
    setpoints = pos_setpoints + neg_setpoints 

def odom_callback(msg):
    global current_point
    current_point = msg.pose.pose.position

def is_at_setpoint(target_x, target_y):
    distance = math.sqrt((target_x - current_point.x)**2 + (target_y - current_point.y)**2)
    return distance < tolerance

def move_to_next_setpoint(pub):
    global setpoint_index, direction, state

    target_x, target_y = setpoints[setpoint_index]

    if state == MOVING:
        setpoint_msg = Pose2D()
        setpoint_msg.x = target_x
        setpoint_msg.y = target_y
        setpoint_msg.theta = 0.0 
        pub.publish(setpoint_msg)

        if is_at_setpoint(target_x, target_y):
            rospy.loginfo(f"Reached setpoint {setpoint_index}: ({target_x}, {target_y})")
            state = ARRIVED  

    elif state == ARRIVED:
        rospy.sleep(1)  

        setpoint_index += direction
        if setpoint_index == len(setpoints):
            direction = -1  
            setpoint_index = len(setpoints) - 1  
        elif setpoint_index == -1: 
            direction = 1  
            setpoint_index = 0  

        state = MOVING  

def main():
    rospy.init_node("robot_navigation_node")
    pub = rospy.Publisher("/setpoint", Pose2D, queue_size=10)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    generate_setpoints()  
    rate = rospy.Rate(10)  

    while not rospy.is_shutdown():
        move_to_next_setpoint(pub)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
