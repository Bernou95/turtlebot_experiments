#ifndef TURTLEBOT3_CONTROLLER_HPP
#define TURTLEBOT3_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

class TurtleBot3Controller : public rclcpp::Node
{
public:
    TurtleBot3Controller();

    void move_forward(double speed = 0.2);
    void rotate(double angular_speed = 0.5);
    void stop();

    float get_minimum_laser_range() const;
    void avoid_obstacles();
    void follow_path();  // Placeholder for future implementation

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::Twist cmd_vel_msg_;
    sensor_msgs::msg::LaserScan current_scan_;
    nav_msgs::msg::Odometry current_odom_;

    bool scan_received_;
    bool odom_received_;

    double linear_speed_;
    double angular_speed_;
    double obstacle_distance_threshold_;

};

#endif  // TURTLEBOT3_CONTROLLER_HPP
