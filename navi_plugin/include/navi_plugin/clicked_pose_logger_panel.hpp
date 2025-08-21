// navi_plugin/clicked_pose_logger_panel.hpp

#ifndef NAVI_PLUGIN__CLICKED_POSE_LOGGER_PANEL_HPP_
#define NAVI_PLUGIN__CLICKED_POSE_LOGGER_PANEL_HPP_

#pragma once

// RViz panel base
#include <rviz_common/panel.hpp>

// Qt
#include <QTimer>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <qdesktopservices.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>

// TF2 (for quaternion → RPY)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STL
#include <vector>
#include <fstream>

namespace navi_plugin
{

class ClickedPoseLoggerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ClickedPoseLoggerPanel(QWidget* parent = nullptr);
  void onInitialize() override;

  using ComputePathThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using GoalHandleCPTP = rclcpp_action::ClientGoalHandle<ComputePathThroughPoses>;
  using PoseS  = geometry_msgs::msg::PoseStamped;
  using PoseVec= std::vector<PoseS>;

private Q_SLOTS:
  void onOpenCsvClicked();
  void onImportCsvClicked();
  void onSaveClicked();
  void onDeleteLastClicked();
  void onClearClicked();
  void onPreviewClicked();
  void onClearPathClicked();

private:
  // Subscriber callbacks
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // Core routines
  void addPose(double x, double y, double z,
               double roll, double pitch, double yaw);
  void publishPoses();
  void updateLabel();
  void saveAllPosesToCSV();
  void flushCSV();
  void clearPreviewPath();
  
  // File I/O 
  bool saveFile(const std::string& file_path);
  bool loadFile(const std::string& file_path);

   // 내부 헬퍼
  bool ensurePlannerReady_();
  bool buildGoal_(ComputePathThroughPoses::Goal & goal, QString* reason = nullptr) const;


  // — Member variables —
  QTimer*                                         ros_spin_timer_{nullptr};
  PoseVec poses_;

  rclcpp::Node::SharedPtr                         node_;
    
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr      waypoint_pub_;
  
  std::string                                     fixed_frame_{"map"}; // Default Frame ID

  rclcpp_action::Client<ComputePathThroughPoses>::SharedPtr planner_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preview_pub_;
  std::string input_frame_id_{"map"};
  std::string planner_server_name_{"/planner_server/compute_path_through_poses"};
  std::string preview_topic_{"/clicked_pose_logger_panel/preview_path"};

  std::string                                     csv_path_{"/tmp/custom_poses.csv"};
  std::ofstream                                   csv_file_;

  bool use_explicit_start_{false};
  PoseS  explicit_start_;
  
  // UI widgets
  QLabel*       current_file_label_{nullptr};
  QLabel*       last_point_label_{nullptr};
  QPushButton*  import_btn_{nullptr};
  QPushButton*  save_btn_{nullptr};
  QPushButton*  delete_last_btn_{nullptr};
  QPushButton*  clear_btn_{nullptr};
  QPushButton*  open_current_btn_{nullptr};
  QPushButton*  preview_btn_{nullptr};
  QPushButton*  clear_path_btn_{nullptr};
  QMessageBox*  ask_clear_{nullptr};
};

} // namespace navi_plugin

#endif // NAVI_PLUGIN__CLICKED_POSE_LOGGER_PANEL_HPP_