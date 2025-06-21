#ifndef TURTLEBOT3_MODEL_HPP
#define TURTLEBOT3_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tracking/pid_controller.hpp"

namespace tracking{



class TurtleBot3Model : public rclcpp::Node
{
public:
    TurtleBot3Model();
    //void update_state(double dt);
    void move(double linear_velocity, double angular_velocity);
    void stop();
    void set_position(double x, double y, double theta);
    //void update_acceleration(double linear_accel, double angular_accel);

    // Getter functions for position, velocity, acceleration
    double get_x() const { return x_; }
    double get_y() const { return y_; }
    double get_theta() const { return theta_; }
    double get_vx() const { return vx_; }
    double get_vy() const { return vy_; }
    double get_vtheta() const { return vtheta_; }
    double get_ax() const { return ax_; }
    double get_ay() const { return ay_; }
    double get_atheta() const { return atheta_; }
    std::pair<double,double> get_alpha() const { return alpha_;}

    // PID controllers for linear and angular movement
    void set_pid_parameters(double linear_Kp, double linear_Ki, double linear_Kd, 
                             double angular_Kp, double angular_Ki, double angular_Kd);
    void move_with_pid(double linear_setpoint, double angular_setpoint, double dt);

    // Laser integration
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Obstacle avoidance based on laser data
    void avoid_obstacles();

    // Calculate point Alpha (α) displaced by l from center of mass
    void calculate_alpha(double l);

    void track_target(double dt);

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q);

private:
    // State: position, velocity, acceleration
    double x_, y_, theta_;
    double vx_, vy_, vtheta_;
    double ax_, ay_, atheta_;
    double l_;  // Position of point Alpha (α)
    std::pair<double, double> alpha_;

    // Laser sensor data
    sensor_msgs::msg::LaserScan current_scan_;
    bool scan_received_;
    
    // Odometry data
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry current_odom_;
    bool odom_received_;

    // PID controllers for linear and angular movement
    PIDController linear_pid_;
    PIDController angular_pid_;

    // Parameters for obstacle avoidance
    bool use_laser_;

    // ROS Publisher
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    geometry_msgs::msg::TwistStamped cmd_vel_msg_;

    // Tracking part
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
    geometry_msgs::msg::Point current_target_;
    bool target_received_;

    void controlLoop();  // callback function for the timer

    rclcpp::TimerBase::SharedPtr timer_;  // timer handle

    
    
    
};
}

#endif  // TURTLEBOT3_MODEL_HPP
