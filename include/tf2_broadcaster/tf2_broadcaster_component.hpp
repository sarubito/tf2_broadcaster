#ifndef TF2_BROADCASTER_COMPONENT_HPP_
#define TF2_BROADCASTER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TF2_BROADCASTER_COMPONENT_EXPORT __attribute__((dllexport))
#define TF2_BROADCASTER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define TF2_BROADCASTER_COMPONENT_EXPORT __declspec(dllexport)
#define TF2_BROADCASTER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef TF2_BROADCASTER_COMPONENT_BUILDING_DLL
#define TF2_BROADCASTER_COMPONENT_PUBLIC \
  TF2_BROADCASTER_COMPONENT_EXPORT
#else
#define TF2_BROADCASTER_COMPONENT_PUBLIC \
  TF2_BROADCASTER_COMPONENT_IMPORT
#endif
#define TF2_BROADCASTER_COMPONENT_PUBLIC_TYPE \
  TF2_BROADCASTER_COMPONENT_PUBLIC
#define TF2_BROADCASTER_COMPONENT_LOCAL
#else
#define TF2_BROADCASTER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define TF2_BROADCASTER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define TF2_BROADCASTER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define TF2_BROADCASTER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define TF2_BROADCASTER_COMPONENT_PUBLIC
#define TF2_BROADCASTER_COMPONENT_LOCAL
#endif
#define TF2_BROADCASTER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "
#endif

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace tf2_bradcaster_component
{
  class TF2BroadCasterComponent : public rclcpp::Node
  {
    public:
      TF2_BROADCASTER_COMPONENT_PUBLIC
      explicit TF2BroadCasterComponent(const rclcpp::NodeOptions & options);
      virtual ~TF2BroadCasterComponent(void);

    private:
      void publish_tf(void);
      void OdometryToTransformed(void);
      void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr data);
      void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data);

      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
      rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
      rclcpp::TimerBase::SharedPtr timer_;

      nav_msgs::msg::Odometry robot_odometry_;
      geometry_msgs::msg::TransformStamped odom_transformed;
      geometry_msgs::msg::PoseWithCovarianceStamped initialpose_;

      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

      bool initialize = false;

  };
}

#endif