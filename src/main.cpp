#include "rclcpp/rclcpp.hpp"
#include "tracking/turtlebot3_model.hpp"  // Adjust with your actual include path

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<tracking::TurtleBot3Model>();

    //node->track_target(0.25);
    rclcpp::spin(node);
    
    
    rclcpp::shutdown();

    return 0;
}