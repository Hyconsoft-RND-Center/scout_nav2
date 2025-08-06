// include <navi_plugin/waypoint_navigator_pane.hpp>
#ifndef NAVI_PLUGIN_WAYPOINT_NAVIGATOR_PANEL_HPP
#define NAVI_PLUGIN_WAYPOINT_NAVIGATOR_PANEL_HPP  

#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
// #include <Q_OBJECT>
#include <QPushButton>
#include <QVBoxLayout>
#include <fstream>
#include <rviz_common/display_context.hpp>
#include <QTimer>
#include <QLabel>
#include <pluginlib/class_list_macros.hpp>
#include <qfiledialog.h>

namespace navi_plugin
{

class WaypointNavigatorPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  WaypointNavigatorPanel(QWidget* parent = nullptr);

  void onInitialize() override;

private slots:
  void onLoadCsvClicked();      // CSV 좌표 불러오기
  void onStartNavigationClicked(); // 여러 pose 주행 시작
  void onStopNavigationClicked();  // 중지/취소

private:
  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle_;

  // 좌표 데이터
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  QString csv_path_{"/tmp/clicked_points.csv"};

  // UI
  QLabel* status_label_;
  QPushButton* load_csv_button_;
  QPushButton* start_nav_button_;
  QPushButton* stop_nav_button_;

  // 기능
  void updateStatus(const QString& msg);
  bool loadWaypointsFromCsv(const QString& path);
  void sendNavigationGoal();
  void cancelNavigationGoal();
};

} // namespace navi_plugin

#endif // NAVI_PLUGIN_WAYPOINT_NAVIGATOR_PANEL_HPP
// End of file: src/scout_simulation/navi_plugin/include/navi_plugin/waypoint_navigator_panel.hpp
// This file defines the WaypointNavigatorPanel class, which is a custom RViz panel for navigating through waypoints using ROS2 navigation actions. It includes methods for loading waypoints from a CSV file, starting and stopping navigation, and updating the UI status. The class uses ROS2 action clients to communicate with the navigation stack and manage the navigation goals. The panel is designed to be integrated into RViz, providing a user interface for waypoint navigation in a robotics simulation or application. The class inherits 