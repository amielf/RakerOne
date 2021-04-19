#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <sstream>
#include <string.h>
#include <cmath>

//ros::Publisher pub_pose_;

void odometryCallback_(const nav_msgs::Odometry::ConstPtr msg) {
    geometry_msgs::Pose pose3d;
    pose3d.position.x = msg->pose.pose.position.x;
    pose3d.position.y = msg->pose.pose.position.y;
    pose3d.position.z = msg->pose.pose.position.z;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double pitch_d = pitch* (180.0/3.141592653589793238463);
    //pose3d.orientation.pitch = pitch;
    //pose3d.orientation.theta = yaw;

    std::string p = std::to_string(pitch_d);
    char* ps = const_cast<char*>(p.c_str());

    ROS_INFO("Hello, World!");
    ROS_INFO("Pitch (degrees): %s", ps);
    //pub_pose_.publish(pose2d);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tilt_node");
    
    ros::NodeHandle nh_;
    
    ros::Subscriber sub_odom_ = nh_.subscribe("odometry/filtered", 1, odometryCallback_);
    //pub_pose_ = nh_.advertise<geometry_msgs::Pose2D>("pose2d", 1);
    
    ros::spin();
    
    return 0;
}
