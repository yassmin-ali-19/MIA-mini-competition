#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "velocity_test");

    // Create a NodeHandle
    ros::NodeHandle nh;

    // Create a publisher to the /cmd_vel topic
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Set a rate of 10 Hz
    ros::Rate rate(10);

    while (ros::ok()) {
        // Create a Twist message to hold velocity data
        geometry_msgs::Twist vel_msg;

        // Set a constant forward velocity
        vel_msg.linear.x = 0.5;  // Move forward at 0.5 m/s
        vel_msg.angular.z = 0.0; // No rotation

        // Publish the velocity message
        vel_pub.publish(vel_msg);

        // Log the command for debugging
        ROS_INFO("Publishing velocity command: linear.x = %f, angular.z = %f", vel_msg.linear.x, vel_msg.angular.z);

        // Sleep to maintain the loop rate
        rate.sleep();
    }

    return 0;
}
