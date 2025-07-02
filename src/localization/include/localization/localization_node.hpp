#include <rclcpp/rclcpp.hpp>

class Localization : public rclcpp::Node
{
public:
    Localization();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;

    bool first_odom_received_ = false;
    geometry_msgs::msg::Pose prev_odom_pose_;
    geometry_msgs::msg::Pose2D estimated_pose_;
};