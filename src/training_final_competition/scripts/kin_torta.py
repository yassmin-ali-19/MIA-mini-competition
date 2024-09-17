#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import  Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

class kinematicScript:
    def __init__(self):
        self.lx = 0.175
        self.ly = 0.075
        
        self.wheel_radius = 0.04
        
        self.linear = [0,0,0]
        self.angular = [0,0,0]
        self.pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.rate = rospy.Rate(20)

    def mecanum_4_vel(self, Vx, Vy, W):
        
        scalar = 1/self.wheel_radius
        
        sum = self.lx + self.ly

        fixed_array = np.array([[1, -1, -sum],
                                [1,  1,  sum],
                                [1,  1, -sum],
                                [1, -1,  sum]])
        
        input_array = np.array([[Vx],
                                [Vy],
                                [W]])

        outside_matrix = (60/(2*np.pi))*np.matmul(scalar * fixed_array, input_array)

        return outside_matrix

    def callback(self,msg):
        self.linear = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.angular = [msg.angular.x, msg.angular.y, msg.angular.z]

        wheel_vel = Float32MultiArray()
        wheel_vel.data = self.mecanum_4_vel(self.linear[0], self.linear[1], self.angular[2])

        self.pub.publish(wheel_vel)

def main():
    rospy.init_node('cmd_vel_listener')

    kinematic = kinematicScript()

    while not rospy.is_shutdown():
        kinematic.rate.sleep()

if __name__ == '__main__':
    main()