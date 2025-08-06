// navi_plugin/pose2d_tool.hpp
#pragma once
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace navi_plugin {

class Pose2DTool : public rviz_default_plugins::tools::PoseTool {
  Q_OBJECT
public:
  Pose2DTool();
  ~Pose2DTool() override = default;

  void onPoseSet(double x, double y, double theta) override;

protected:
  void onInitialize() override;
  void publishPose(double x, double y, double theta);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::string topic_name_{"clicked_pose2d"};
  std::string frame_id_{"map"};
};

} // namespace navi_plugin
