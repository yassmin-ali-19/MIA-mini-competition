#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include <chrono>

#define abs(a) (((a) < 0) ? (-a) : (a))

namespace gm = geometry_msgs;
namespace nav = nav_msgs;

class WaypointGenerator {
public:
    WaypointGenerator() {
        // Manually assign waypoints (increased number of waypoints)

        waypoints[0].x = 1.0; waypoints[0].y = 0.0; waypoints[0].theta = 0.0;
        waypoints[1].x = 0.4; waypoints[1].y = 0.0; waypoints[1].theta = 0.0;
        waypoints[2].x = 0.4; waypoints[2].y = -0.3; waypoints[2].theta = 0.0;
        waypoints[3].x = 1.0; waypoints[3].y = -0.3; waypoints[3].theta = 0.0;
        waypoints[4].x = 0.4; waypoints[4].y = -0.3; waypoints[4].theta = 0.0;
        waypoints[5].x = 0.4; waypoints[5].y = -0.6; waypoints[5].theta = 0.0; // New waypoint
        waypoints[6].x = 1.0; waypoints[6].y = -0.5; waypoints[6].theta = 0.0; // New waypoint
        waypoints[7].x = 0.4; waypoints[7].y = -0.5; waypoints[7].theta = 0.0; // New waypoint
        waypoints[8].x = 1.0; waypoints[7].y = -0.8; waypoints[7].theta = 0.0; // New waypoint

        waypoints[9].x = 0.4; waypoints[0].y = 0.0; waypoints[0].theta = 0.0;
        waypoints[10].x = 1.0; waypoints[0].y = 0.0; waypoints[0].theta = 0.0;
        waypoints[11].x = 0.4; waypoints[1].y = 0.0; waypoints[1].theta = 0.0;
        waypoints[12].x = 0.4; waypoints[2].y = 0.3; waypoints[2].theta = 0.0;
        waypoints[13].x = 1.0; waypoints[3].y = 0.3; waypoints[3].theta = 0.0;
        waypoints[14].x = 0.4; waypoints[4].y = 0.3; waypoints[4].theta = 0.0;
        waypoints[15].x = 0.4; waypoints[5].y = 0.6; waypoints[5].theta = 0.0; // New waypoint
        waypoints[16].x = 1.0; waypoints[6].y = 0.6; waypoints[6].theta = 0.0; // New waypoint
        waypoints[17].x = 0.4; waypoints[7].y = 0.6; waypoints[7].theta = 0.0; // New waypoint
        waypoints[18].x = 1.0; waypoints[7].y = 0.8; waypoints[7].theta = 0.0;
        // Add more waypoints as needed...
    }

    gm::Pose2D get_next_waypoint(int index) {
        gm::Pose2D pose;
        if (index >= 0 && index < NUM_WAYPOINTS) {
            pose.x = waypoints[index].x;
            pose.y = waypoints[index].y;
            pose.theta = waypoints[index].theta;
        }
        return pose;
    }

    static const int NUM_WAYPOINTS = 19; // Update to reflect new number of waypoints

private:
    gm::Pose2D waypoints[NUM_WAYPOINTS];
};

struct Data {
    gm::Vector3 linear;
    gm::Vector3 angular;
    gm::Point point;

    void fetch_data(const nav::Odometry::ConstPtr& msg) {
        linear.x = msg->twist.twist.linear.x;
        linear.y = msg->twist.twist.linear.y;
        angular.z = msg->twist.twist.angular.z;

        point.x = msg->pose.pose.position.x;
        point.y = msg->pose.pose.position.y;
        point.z = 0;
    }
};

bool is_at_waypoint(const gm::Pose2D& curr_pose, const gm::Pose2D& waypoint) {
    float threshold = 0.1; // Define a threshold for reaching the waypoint
    return (abs(curr_pose.x - waypoint.x) < threshold && abs(curr_pose.y - waypoint.y) < threshold);
}

int main(int argc, char **argv) {
    Data data;

    gm::Pose2D curr_pose;
    curr_pose.x = 0.0;
    curr_pose.y = 0.0;

    ros::init(argc, argv, "navigator");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 1000, &Data::fetch_data, &data);
    ros::Publisher pub = nh.advertise<gm::Pose2D>("/setpoint", 100);

    // ros::Rate loop_rate(10); // Set a higher rate for responsiveness
    WaypointGenerator waypointGen;
    int index = 0;
    bool waiting = false;

    // Use chrono to handle timing
    auto last_time = std::chrono::steady_clock::now();

    while (ros::ok()) {
        gm::Pose2D next_pose = waypointGen.get_next_waypoint(index);

        // Check if the robot is at the waypoint
        if (is_at_waypoint(curr_pose, next_pose)) {
            if (!waiting) {
                // Start waiting if not already waiting
                last_time = std::chrono::steady_clock::now();
                waiting = true;
            } else {
                // Check if 3 seconds have passed
                auto now = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_time);
                if (duration.count() >= 5) {
                    // Move to the next waypoint
                    index = (index + 1) % WaypointGenerator::NUM_WAYPOINTS;
                    waiting = false; // Reset waiting state
                }
            }
        }

        // Publish the next pose
        pub.publish(next_pose);
        curr_pose = next_pose; // Update current pose (simulated)

        ros::spin();
    }
    return 0;
}
