#include "tf2_broadcaster/tf2_broadcaster_component.hpp"

namespace tf2_bradcaster_component
{
    TF2BroadCasterComponent::TF2BroadCasterComponent(const rclcpp::NodeOptions & options) : Node("tf2_broadcaster_node", options)
    {
        timer_ = this->create_wall_timer(500ms, std::bind(&TF2BroadCasterComponent::publish_tf, this));
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&TF2BroadCasterComponent::odometry_callback, this, std::placeholders::_1));
    }

    void TF2BroadCasterComponent::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr data)
    {
        robot_odometry_ = *data;
    }

    void TF2BroadCasterComponent::OdometryToTransformed(void)
    {
        odom_transformed.transform.translation.x = robot_odometry_.pose.pose.position.x;
        odom_transformed.transform.translation.y = robot_odometry_.pose.pose.position.y;
        odom_transformed.transform.translation.z = robot_odometry_.pose.pose.position.z;
        odom_transformed.transform.rotation = robot_odometry_.pose.pose.orientation;
    }

    void TF2BroadCasterComponent::publish_tf(void)
    {
        tf_broadcaster_ =  std::make_shared<tf2_ros::TransformBroadcaster>(this);
        OdometryToTransformed();
        odom_transformed.header.stamp = robot_odometry_.header.stamp;
        odom_transformed.header.frame_id = "map";
        odom_transformed.child_frame_id = "odom";
        tf_broadcaster_->sendTransform(odom_transformed);
    }

    TF2BroadCasterComponent::~TF2BroadCasterComponent(void){}
}

RCLCPP_COMPONENTS_REGISTER_NODE(tf2_bradcaster_component::TF2BroadCasterComponent)