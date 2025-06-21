#include "turtlebot3_controller.hpp"
#include <algorithm>
#include <limits>

TurtleBot3Controller::TurtleBot3Controller()
    : Node("turtlebot3_controller"), scan_received_(false), odom_received_(false)
{
    declare_parameter("linear_speed", 0.2);
    declare_parameter("angular_speed", 0.5);
    declare_parameter("obstacle_distance_threshold", 0.5);

    linear_speed_ = get_parameter("linear_speed").as_double();
    angular_speed_ = get_parameter("angular_speed").as_double();
    obstacle_distance_threshold_ = get_parameter("obstacle_distance_threshold").as_double();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&TurtleBot3Controller::laser_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&TurtleBot3Controller::odom_callback, this, std::placeholders::_1));
}

void TurtleBot3Controller::move_forward(double speed)
{
    cmd_vel_msg_.linear.x = speed;
    cmd_vel_msg_.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg_);
}

void TurtleBot3Controller::rotate(double angular_speed)
{
    cmd_vel_msg_.linear.x = 0.0;
    cmd_vel_msg_.angular.z = angular_speed;
    cmd_vel_pub_->publish(cmd_vel_msg_);
}

void TurtleBot3Controller::stop()
{
    cmd_vel_msg_ = geometry_msgs::msg::TwistStamped();  // All zeros
    cmd_vel_pub_->publish(cmd_vel_msg_);
}

void TurtleBot3Controller::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    current_scan_ = *msg;
    scan_received_ = true;
}

void TurtleBot3Controller::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_ = *msg;
    odom_received_ = true;
}

float TurtleBot3Controller::get_minimum_laser_range() const
{
    if (!scan_received_) return std::numeric_limits<float>::infinity();
    return *std::min_element(current_scan_.ranges.begin(), current_scan_.ran_*
