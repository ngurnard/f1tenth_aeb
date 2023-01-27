#include "safety_node/safety_node.hpp"

/// TODO: create ROS subscribers and publishers

Safety::Safety() : Node("safety_node"), m_speed(0.0)//, m_thresh(1)
{
    /*
    You should also subscribe to the /scan topic to get the
    sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
    the nav_msgs/Odometry messages

    The subscribers should use the provided odom_callback and 
    scan_callback as callback methods

    NOTE that the x component of the linear velocity in odom is the speed
    */

    // Subscribe to the LaserScan message
    m_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    "/scan", 1, std::bind(&Safety::scan_callback, this, std::placeholders::_1));

    // Subscribe to the Odometry messages
    m_odom = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/ego_racecar/odom", 1, std::bind(&Safety::odom_callback, this, std::placeholders::_1));

    // Create a publisher to drive
    m_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);

    // Make a parameter for the threshold m_thresh
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Threshold value that is used to determine if the iTTC required automatic emergency braking";
    this->declare_parameter("m_thresh", 1.0f, param_desc);  // Speed
}

void Safety::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    // update current speed
    this->m_speed = msg->twist.twist.linear.x; // get the x field of linear
}

void Safety::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
{
    float ttc;
    // float ttc_temp;
    // int hit_count = 0; // to filter out false positives (braking when collision isn't imminent)

    float curr_angle = scan_msg->angle_min;

    for (float curr_range : scan_msg->ranges) // loop through all of the laser scan to calculate iTTC
    {
        // if nan or inf, then continue to the next scan
        if (0
            || std::isnan(curr_range) == true 
            || std::isinf(curr_range) == true 
            || curr_range > scan_msg->range_max
            || curr_range < scan_msg->range_min) {
            
            // Increment the angle
            curr_angle += scan_msg->angle_increment;
            continue;
        }

        // Compute the time to collision
        // float range_rate = std::max(-(m_speed * std::cos(curr_angle)), 0.0);
        float range_rate = -(m_speed * std::cos(curr_angle));

        if (range_rate < 0) { // if going forward
            ttc = curr_range / (-range_rate); // time to collision
        } else {
            // Increment the angle
            curr_angle += scan_msg->angle_increment;
            continue; // continue if divide by zero
        }

        // Use the minimum time to collision
        if (ttc < this->get_parameter("m_thresh").get_parameter_value().get<double>()) {
            // hit_count += 1;

            // publish if the ttc is less than the threshold and if hit_count is enough
            ackermann_msgs::msg::AckermannDriveStamped msg;
            msg.drive.speed = 0.0f;
            m_publisher->publish(msg);
            RCLCPP_ERROR(this->get_logger(), "Collision!"); // INFO, ERROR, WARN
        }

        // Increment the angle
        curr_angle += scan_msg->angle_increment;
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}