#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

class Localization : public rclcpp::Node
{
public:
    Localization();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    bool first_odom_received_ = false;
    geometry_msgs::msg::Pose2D prev_odom_pose_;
    geometry_msgs::msg::Pose2D last_odom_pose_;
    geometry_msgs::msg::Pose2D estimated_pose_;

    bool first_pointcloud_received_ = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_scan_;
    double fitness_threshold_; 
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // For publishing a transform between map and base_link
};