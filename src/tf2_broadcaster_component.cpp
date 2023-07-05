#include "tf2_broadcaster/tf2_broadcaster_component.hpp"

namespace tf2_bradcaster_component
{
    TF2BroadCasterComponent::TF2BroadCasterComponent(const rclcpp::NodeOptions & options) : Node("tf2_broadcaster_node", options)
    {
        timer_ = this->create_wall_timer(500ms, std::bind(&TF2BroadCasterComponent::publish_tf, this));
        odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&TF2BroadCasterComponent::odometry_callback, this, std::placeholders::_1));
        initial_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10 ,std::bind(&TF2BroadCasterComponent::initialpose_callback, this, std::placeholders::_1));
        m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    }

    void TF2BroadCasterComponent::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr data)
    {
        robot_odometry_ = *data;
    }

    void TF2BroadCasterComponent::initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data)
    {
        initialpose_ = *data;
        RCLCPP_INFO(this->get_logger(), "initial pose x : %f", initialpose_.pose.pose.position.x);
        RCLCPP_INFO(this->get_logger(), "initial pose y : %f", initialpose_.pose.pose.position.y);
        RCLCPP_INFO(this->get_logger(), "initial pose z : %f", initialpose_.pose.pose.position.z);
        // try{
        //     t = m_tfBuffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        // } catch(const tf2::TransformException & ex) {
        //     RCLCPP_INFO(this->get_logger(), "error");
        // }

    }

    void TF2BroadCasterComponent::OdometryToTransformed(void)
    {
        odom_transformed_.transform.translation.x = 0.0;
        odom_transformed_.transform.translation.y = 0.0;
        odom_transformed_.transform.translation.z = 0.0;
        odom_transformed_.transform.rotation.x = 0.0;
        odom_transformed_.transform.rotation.y = 0.0;
        odom_transformed_.transform.rotation.z = 0.0;
        odom_transformed_.transform.rotation.w = 1.0;
    }

    void TF2BroadCasterComponent::publish_tf(void)
    {
        tf_broadcaster_ =  std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // tf2::convert(robot_odometry_.pose.pose, txi);
        // geometry_msgs::msg::TransformStamped txi_inv;
        // txi_inv.header.frame_id = "base_footprint";
        // txi_inv.header.stamp = robot_odometry_.header.stamp;
        // tf2::convert(txi.inverse(), txi_inv.transform);

        OdometryToTransformed();
        odom_transformed_.header.stamp = robot_odometry_.header.stamp;
        odom_transformed_.header.frame_id = "map";
        odom_transformed_.child_frame_id = "odom";
        tf_broadcaster_->sendTransform(odom_transformed_);
    }

    TF2BroadCasterComponent::~TF2BroadCasterComponent(void){}
}

RCLCPP_COMPONENTS_REGISTER_NODE(tf2_bradcaster_component::TF2BroadCasterComponent)