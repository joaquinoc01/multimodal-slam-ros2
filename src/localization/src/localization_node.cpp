#include "include/localization_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>  // for tf2::getYaw

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

Localization::Localization() : Node("localization_node")
{
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&Localization::odom_callback, this, std::placeholders::_1));
    pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/odom", 10, std::bind(&Localization::pointcloud_callback, this, std::placeholders::_1));
    previous_scan_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Initialize estimated_pose
    estimated_pose_.x = 0.0;
    estimated_pose_.y = 0.0;
    estimated_pose_.theta = 0.0;

    fitness_threshold_ = 0.001; // For rejecting bad ICP matches
}

void Localization::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    last_odom_pose_.x = msg->pose.pose.position.x;
    last_odom_pose_.y = msg->pose.pose.position.y;

    // Convert quaternion to yaw
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    last_odom_pose_.theta = tf2::getYaw(q);

    if (!first_odom_received_) {
        first_odom_received_ = true;
    }
}

void Localization::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *current_cloud);

    if (!first_pointcloud_received_) {
        *previous_scan_ = *current_cloud;
        prev_odom_pose_ = last_odom_pose_;
        first_pointcloud_received_ = true;
        return;
    }

    // Compute odometry delta from last scan pose
    double dx = last_odom_pose_.x - prev_odom_pose_.x;
    double dy = last_odom_pose_.y - prev_odom_pose_.y;
    double dtheta = last_odom_pose_.theta - prev_odom_pose_.theta;

    // Predict pose using odometry
    float cos_theta = std::cos(estimated_pose_.theta);
    float sin_theta = std::sin(estimated_pose_.theta);

    geometry_msgs::msg::Pose2D predicted_pose;
    predicted_pose.x = estimated_pose_.x + cos_theta * dx - sin_theta * dy;
    predicted_pose.y = estimated_pose_.y + sin_theta * dx + cos_theta * dy;
    predicted_pose.theta = estimated_pose_.theta + dtheta;
    predicted_pose.theta = std::atan2(std::sin(predicted_pose.theta), std::cos(predicted_pose.theta));

    // Run ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(current_cloud);
    icp.setInputTarget(previous_scan_);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    if (icp.hasConverged() && icp.getFitnessScore() < fitness_threshold_) {
        Eigen::Matrix4f transformation = icp.getFinalTransformation();

        double delta_x = transformation(0,3);
        double delta_y = transformation(1,3);
        float delta_theta = std::atan2(transformation(1,0), transformation(0,0));

        float cos_theta = std::cos(estimated_pose_.theta);
        float sin_theta = std::sin(estimated_pose_.theta);

        estimated_pose_.x += cos_theta * delta_x - sin_theta * delta_y;
        estimated_pose_.y += sin_theta * delta_x + cos_theta * delta_y;
        estimated_pose_.theta += delta_theta;
        estimated_pose_.theta = std::atan2(std::sin(estimated_pose_.theta), std::cos(estimated_pose_.theta));
    } else {
        RCLCPP_WARN(this->get_logger(), "Poor transform. Rejecting ICP. Using only odometry.");
        estimated_pose_ = predicted_pose;
    }

    *previous_scan_ = *current_cloud;
    prev_odom_pose_ = last_odom_pose_;  // Update odom ref for next delta
}
