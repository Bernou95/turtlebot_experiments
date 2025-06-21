#include "tracking/turtlebot3_model.hpp"
#include <cmath>  // For trigonometric functions (sin, cos)
namespace tracking{
    

TurtleBot3Model::TurtleBot3Model()
    : Node("turtlebot3_model"), 
      x_(0.0), y_(0.0), theta_(0.0), vx_(0.0), vy_(0.0), vtheta_(0.0),
      ax_(0.0), ay_(0.0), atheta_(0.0),
      scan_received_(false), odom_received_(false),
      linear_pid_(1.0, 0.0, 0.1, 1.0,-1.0),  // Example PID gains for linear
      angular_pid_(1.0, 0.0, 0.1, 1.0,-1.0),  // Example PID gains for angular
      use_laser_(true)  // Default to using laser data
{
    use_laser_ = this->declare_parameter<bool>("use_laser", false);

    auto linear_pid_kp = this->declare_parameter<double>("linear_pid.kp", 1.0);
    auto linear_pid_ki = this->declare_parameter<double>("linear_pid.ki", 0.0);
    auto linear_pid_kd = this->declare_parameter<double>("linear_pid.kd", 0.0);

    auto angular_pid_kp = this->declare_parameter<double>("angular_pid.kp", 1.0);
    auto angular_pid_ki = this->declare_parameter<double>("angular_pid.ki", 0.0);
    auto angular_pid_kd = this->declare_parameter<double>("angular_pid.kd", 0.0);
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    
    this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&TurtleBot3Model::laser_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&TurtleBot3Model::odom_callback, this, std::placeholders::_1));
    
    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "target_pose", 10,
    [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        current_target_ = *msg;
        target_received_ = true;
    });

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&TurtleBot3Model::controlLoop, this));

}


/*void TurtleBot3Model::update_state(double dt)
{
    // Simple kinematic model (using Euler integration)
    x_ += vx_ * dt;
    y_ += vy_ * dt;
    theta_ += vtheta_ * dt;

    vx_ += ax_ * dt;
    vy_ += ay_ * dt;
    vtheta_ += atheta_ * dt;
}*/

void TurtleBot3Model::move(double linear_velocity, double angular_velocity)
{
    cmd_vel_msg_.header.frame_id = "base_link";
    //cmd_vel_msg_.header.stamp = this->get_clock()->now();
    cmd_vel_msg_.twist.linear.x = linear_velocity;
    cmd_vel_msg_.twist.angular.z = angular_velocity;
    cmd_vel_pub_->publish(cmd_vel_msg_);
    //RCLCPP_INFO(this->get_logger(), "Cmd vel should work!");
}

void TurtleBot3Model::stop()
{
    cmd_vel_msg_ = geometry_msgs::msg::TwistStamped();  // Zero velocities
    cmd_vel_pub_->publish(cmd_vel_msg_);
}

void TurtleBot3Model::set_position(double x, double y, double theta)
{
    x_ = x;
    y_ = y;
    theta_ = theta;
}

/*void TurtleBot3Model::update_acceleration(double linear_accel, double angular_accel)
{
    ax_ = linear_accel;
    ay_ = 0.0;  // Assuming we're working in a 2D plane
    atheta_ = angular_accel;
}*/

void TurtleBot3Model::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    current_scan_ = *msg;
    scan_received_ = true;
}

void TurtleBot3Model::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Extract robot position and orientation
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = getYawFromQuaternion(msg->pose.pose.orientation);

    current_odom_ = *msg;
    set_position(x,y,yaw);
    calculate_alpha(l_);
    odom_received_ = true;
}

void TurtleBot3Model::move_with_pid(double linear_setpoint, double angular_setpoint, double dt)
{
    double linear_velocity = linear_pid_.compute(linear_setpoint, vx_, dt);
    double angular_velocity = angular_pid_.compute(angular_setpoint, vtheta_, dt);

    move(linear_velocity, angular_velocity);
}

void TurtleBot3Model::set_pid_parameters(double linear_Kp, double linear_Ki, double linear_Kd, 
                                          double angular_Kp, double angular_Ki, double angular_Kd)
{
    linear_pid_ = PIDController(linear_Kp, linear_Ki, linear_Kd, 1.0, -1.0);
    angular_pid_ = PIDController(angular_Kp, angular_Ki, angular_Kd, 1.0, -1.0);
}

void TurtleBot3Model::avoid_obstacles()
{
    if (!scan_received_ || !use_laser_) return;

    float min_range = *std::min_element(current_scan_.ranges.begin(), current_scan_.ranges.end());

    if (min_range < 0.5) // Simple threshold
    {
        RCLCPP_INFO(this->get_logger(), "Obstacle detected! Rotating...");
        move(0.0, 0.5); // Rotate
    }
    else
    {
        move(0.2, 0.0); // Move forward
    }
}

void TurtleBot3Model::calculate_alpha(double l)
{
    alpha_.first = x_ + l * cos(theta_);
    alpha_.second = y_ + l * sin(theta_);
    RCLCPP_INFO(this->get_logger(), "Alpha (x, y): (%f, %f)", alpha_.first, alpha_.second);
}

void TurtleBot3Model::track_target(double dt)
{
    //RCLCPP_INFO(this->get_logger(), "Start track step!");
    if (!target_received_ || !odom_received_) return;

    // Calculate distance and angle to target
    double dx = current_target_.x - x_;
    double dy = current_target_.y - y_;
    double distance = std::sqrt(dx * dx + dy * dy);
    double target_theta = std::atan2(dy, dx);
    double heading_error = target_theta - theta_;

    // Normalize angle to [-π, π]
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;

    // PID tracking
    double linear_cmd = linear_pid_.compute(distance, 0.0, dt);
    double angular_cmd = angular_pid_.compute(heading_error, 0.0, dt);

    // Obstacle check
    if (use_laser_ && scan_received_) {
        float min_range = *std::min_element(current_scan_.ranges.begin(), current_scan_.ranges.end());
        if (min_range < 0.4) {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected during tracking. Rotating...");
            move(0.0, 0.5);  // Basic avoidance
            return;
        }
    }

    move(linear_cmd, angular_cmd);
}

double TurtleBot3Model::getYawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

void TurtleBot3Model::controlLoop()
{
  // Your periodic control code here
  //RCLCPP_INFO(this->get_logger(), "Control loop tick");
  track_target(.1);
}

}



