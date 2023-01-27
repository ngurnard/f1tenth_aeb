#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.h"


class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    // Default Constructor
    Safety();

private:
    double m_speed; // m_ is being used to denote member variables
    // float m_thresh; 
    
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_brake;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_publisher;

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
};