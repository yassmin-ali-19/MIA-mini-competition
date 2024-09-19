#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include <chrono>

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define clamp(v, l, h) min(max(v, l), h)

namespace gm = geometry_msgs;
namespace stdm = std_msgs;
namespace nav = nav_msgs;
namespace chrono = std::chrono;

const float MAX_SPEED = 0.45;

enum Clamp { NO_CLAMP, CLAMP };

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

float normalize_angle(float angle) {
    angle = fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0) angle += 2.0 * M_PI;
    angle -= M_PI;
    return angle;
}


struct AxisRot {
    double yaw = 0, pitch = 0, roll = 0;
    AxisRot() {};
    AxisRot(double yaw, double pitch, double roll) : yaw(yaw), pitch(pitch), roll(roll) {}
    AxisRot(tf2::Quaternion q) {
        tf2::Matrix3x3 matrix(q);
        matrix.getRPY(roll, pitch, yaw);
    }
};




struct FetchData {
    float set_x, set_y, set_theta;
    tf2::Quaternion q;
    gm::Point p, setPoint;
    gm::Vector3 linear;
    gm::Vector3 angular;
    gm::Pose pose;

    void fetch_data(const nav::Odometry::ConstPtr& msg) {
        q.setX(msg->pose.pose.orientation.x);
        q.setY(msg->pose.pose.orientation.y);
        q.setZ(msg->pose.pose.orientation.z);
        q.setW(msg->pose.pose.orientation.w);

        p.x = msg->pose.pose.position.x;
        p.y = msg->pose.pose.position.y;
        p.z = msg->pose.pose.position.z;

        linear.x = msg->twist.twist.linear.x;
        linear.y = msg->twist.twist.linear.y;
        linear.z = msg->twist.twist.linear.z;

        angular.x = msg->twist.twist.angular.x;
        angular.y = msg->twist.twist.angular.y;
        angular.z = msg->twist.twist.angular.z;

        pose = msg->pose.pose;

    }
    void FetchPoint(const gm::Pose2D::ConstPtr& msg) {
        setPoint.x = msg->x;
        setPoint.y = msg->y;
        setPoint.z = 0;
        set_theta = normalize_angle(msg->theta);
    }
};


struct Pid {
private:
    float kp, ki, kd;
    float accum;
    float prev_err;
    TimePoint prev_time;

public:

    // void fetch(const gm::Vector3::ConstPtr& msg) {
    //     kp = msg->x;
    //     ki = msg->y;
    //     kd = msg->z;
    // }
    Pid(float kp, float ki, float kd): kp(kp), ki(ki), kd(kd) {}

    float get_output(const float target, const float current) {
        float err = target - current;
        auto dt = (float)(Clock::now() - prev_time).count();

        float p = kp * err;
        float d = kd * (prev_err - err) / dt;
        accum += err * dt;
        accum = clamp(accum, -MAX_SPEED, MAX_SPEED);

        float i = ki * accum;
       
        prev_err = err;
        return p + d + i;
    }
};

void publish_output(const gm::Twist& msg, ros::Publisher& pub) {
    pub.publish(msg);
}

geometry_msgs::Point transformToRobotFrame(
    const geometry_msgs::Point& setPoint, const geometry_msgs::Pose& robotPose) {
        
    geometry_msgs::Point transformed_point;
    tf2::Transform robot_transform(
        tf2::Quaternion(
            robotPose.orientation.x, 
            robotPose.orientation.y, 
            robotPose.orientation.z, 
            robotPose.orientation.w
        ),
        tf2::Vector3(
            robotPose.position.x, 
            robotPose.position.y, 
            robotPose.position.z
        )
    );

    tf2::Transform global_point_transform(
        tf2::Quaternion(0, 0, 0, 1),  
        tf2::Vector3(
            setPoint.x, 
            setPoint.y, 
            setPoint.z
        )
    );

    tf2::Transform relative_transform = robot_transform.inverse() * global_point_transform;
    tf2::Vector3 relative_position = relative_transform.getOrigin();

    transformed_point.x = relative_position.x();
    transformed_point.y = relative_position.y();
    transformed_point.z = 0;

    return transformed_point;
}


int main(int argc, char **argv) {
    FetchData data;
    Pid x_pid(1,0.01,0.1);
    Pid y_pid(1,0.01,0.1);
    Pid theta_pid(1,0.01,0.1);

    

    ros::init(argc, argv, "PID");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 1000, &FetchData::fetch_data, &data);
    ros::Subscriber sub2 = nh.subscribe("/setpoint", 100, &FetchData::FetchPoint, &data);
    // ros::Subscriber sub3 = nh.subscribe("/set_param", 100, &Pid::fetch, &x_pid);
    ros::Publisher pub = nh.advertise<gm::Twist>("/cmd_vel", 1000);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        gm::Point robot_set_point = transformToRobotFrame(data.setPoint, data.pose);
        float set_x = robot_set_point.x;
        float set_y = robot_set_point.y;

        auto rot_rpy = AxisRot(data.q);
        auto x_out = min(MAX_SPEED, x_pid.get_output(set_x, data.linear.x));
        auto y_out = min(MAX_SPEED, y_pid.get_output(set_y, data.linear.y));
        auto theta_out = min(MAX_SPEED, theta_pid.get_output(data.set_theta, rot_rpy.yaw));

        gm::Twist msg;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = theta_out;
        msg.linear.x = x_out;
        msg.linear.y = y_out;
        msg.linear.z = 0.0;

        publish_output(msg, pub);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}