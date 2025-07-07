#include "lidar_frontend/scan_subscriber.hpp"

#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

ScanSubscriber::ScanSubscriber() : Node("scan_subscriber")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&ScanSubscriber::scan_callback, this, std::placeholders::_1));
    pc_publisher_ = this ->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/scan_pointcloud", 10);
    RCLCPP_INFO(this->get_logger(), "Publisher created: scan_publisher");
}

void ScanSubscriber::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::vector<std::pair<float, float>> points;

    for(size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float r = msg->ranges[i];
      if (std::isfinite(r) && r > 0.0f)
      {
        float theta = msg->angle_min + i * msg->angle_increment;
        float x = r * std::cos(theta);
        float y = r * std::sin(theta);

        points.emplace_back(x, y);
      }
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.header.frame_id = msg->header.frame_id;

    for (const auto& [x, y] : points) {
        pcl_cloud.points.emplace_back(x, y, 0.0f);
    }

    // Convert to ROS2 message
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(pcl_cloud, ros_cloud);
    ros_cloud.header.stamp = msg->header.stamp;

    pc_publisher_->publish(ros_cloud);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanSubscriber>());
  rclcpp::shutdown();
  return 0;
}