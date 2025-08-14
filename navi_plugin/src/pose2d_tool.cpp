#include "navi_plugin/pose2d_tool.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace navi_plugin {

Pose2DTool::Pose2DTool() { shortcut_key_ = 'w'; }

void Pose2DTool::onInitialize() {
  rviz_default_plugins::tools::PoseTool::onInitialize();
  auto abstraction = context_->getRosNodeAbstraction().lock();
  auto raw = abstraction->get_raw_node();
  node_ = abstraction->get_raw_node();
  publisher_ = raw->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, 10);
}

void Pose2DTool::onPoseSet(double x, double y, double theta) {
  theta = std::atan2(std::sin(theta), std::cos(theta));
  publishPose(x, y, theta);
}

void Pose2DTool::publishPose(double x, double y, double theta) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = node_->now();
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;
  tf2::Quaternion q; q.setRPY(0,0,theta);
  msg.pose.orientation = tf2::toMsg(q);
  publisher_->publish(msg);
}

} // namespace navi_plugin

PLUGINLIB_EXPORT_CLASS(navi_plugin::Pose2DTool, rviz_common::Tool)
