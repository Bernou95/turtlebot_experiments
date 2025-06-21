#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>

class TargetPublisher : public rclcpp::Node
{
public:
    TargetPublisher() : Node("target_publisher")
    {
        pub_ = this->create_publisher<geometry_msgs::msg::Point>("target_pose", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TargetPublisher::publish_target, this));
        angle_ = 0.0;
    }

private:
    void publish_target()
    {
        geometry_msgs::msg::Point point;
        point.x = 1.5 * cos(angle_);
        point.y = 1.5 * sin(angle_);
        point.z = 0.0;

        pub_->publish(point);
        angle_ += 0.05;
    }

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double angle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetPublisher>());
    rclcpp::shutdown();
    return 0;
}
