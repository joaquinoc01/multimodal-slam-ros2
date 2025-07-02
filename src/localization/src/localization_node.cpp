#include "include/localization_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>  // for tf2::getYaw

Localization::Localization() : Node("localization_node")
{
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&Localization::odom_callback, this, std::placeholders::_1));
    pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/odom", 10, std::bind(&Localization::pointcloud_callback, this, std::placeholders::_1));
}

void Localization::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Convert quaternion to yaw
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w,
    );
    double theta = tf2::getYaw(q);

    if (!first_odom_received_)
    {
        first_odom_received_ = true;
        prev_odom_pose_.x = x;
        prev_odom_pose_.y = y;
        prev_odom_pose_.theta = theta;
        return;
    }
    
    double dx = x - prev_odom_pose.x;
    double dy = y - prev_odom_pose.y;
    double dtheta = theta - prev_odom_pose.theta;

    estimate_pose.x += dx;
    estimate_pose.y += dy;
    estimate_pose.theta += dtheta;

    // Save current for next step
    prev_odom_pose.x = x;
    prev_odom_pose.y = y;
    prev_odom_pose.theta = theta;
}

void Localization::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    
}