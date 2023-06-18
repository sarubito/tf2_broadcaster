#include "tf2_broadcaster/tf2_broadcaster_component.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto tf2_broadcaster_component = std::make_shared<tf2_bradcaster_component::TF2BroadCasterComponent>(rclcpp::NodeOptions());
    exec.add_node(tf2_broadcaster_component);
    exec.spin();
    rclcpp::shutdown();
}