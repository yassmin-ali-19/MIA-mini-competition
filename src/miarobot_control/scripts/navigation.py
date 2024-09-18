#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Pose2D


x_min = 0.5
x_max = 1.5
y_start = 0.15  # Starting y-position, 15 cm away from the edge
y_max = 1.7
y_increment = 0.3
penalty_zones = [(0, 0), (1.8, 0)]  # Corners as penalty zones

# Function to publish the next setpoint (x, y, theta)
def move_robot(pub, x, y, theta):
    setpoint = Pose2D(x=x, y=y, theta=theta)
    pub.publish(setpoint)

def robot_navigation():
    rospy.init_node('robot_navigation_node', anonymous=True)
    setpoint_pub = rospy.Publisher('setpoint_topic', Pose2D, queue_size=10)
    rate = rospy.Rate(10)

    x = x_min
    y = y_start
    theta = 0.0  

    direction = 1 

    while not rospy.is_shutdown():
        if (x >= x_max and direction == 1) or (x <= x_min and direction == -1):
            # Increment y and switch direction
            y += y_increment
            direction *= -1  # Reverse the direction along x
            if y > y_max:
                y = y_start  # If y exceeds the map, reset to start and repeat

        # Move the robot to the next point along x
        x += direction * 0.1  # Adjust the step size for smooth movement

        # Ensure the robot doesn't enter the penalty zones
        if not any(zone[0] <= x <= zone[0] + 0.2 and zone[1] <= y <= zone[1] + 0.2 for zone in penalty_zones):
            move_robot(setpoint_pub, x, y, theta)

        rate.sleep()

if __name__ == '__main__':
    try:
        robot_navigation()
    except rospy.ROSInterruptException:
        pass
