// src/waypoint_navigator_panel.cpp
#include "navi_plugin/waypoint_navigator_panel.hpp"

namespace navi_plugin 
{

WaypointNavigatorPanel::WaypointNavigatorPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  node_ = std::make_shared<rclcpp::Node>("waypoint_navigator_panel_node");
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
        node_, "navigate_through_poses");
    // UI 초기화
    QVBoxLayout* layout_ = new QVBoxLayout(this);
    status_label_ = new QLabel("Status: Idle", this);
    load_csv_button_ = new QPushButton("Load CSV", this);
    start_nav_button_ = new QPushButton("Start Navigation", this);
    stop_nav_button_ = new QPushButton("Stop Navigation", this);
    connect(load_csv_button_, &QPushButton::clicked, this, &WaypointNavigatorPanel::onLoadCsvClicked);
    connect(start_nav_button_, &QPushButton::clicked, this, &WaypointNavigatorPanel::onStartNavigationClicked);
    connect(stop_nav_button_, &QPushButton::clicked, this, &WaypointNavigatorPanel::onStopNavigationClicked);
    layout_->addWidget(status_label_);
    layout_->addWidget(load_csv_button_);
    layout_->addWidget(start_nav_button_);
    layout_->addWidget(stop_nav_button_);
    setLayout(layout_);
}

void WaypointNavigatorPanel::onInitialize()
{
  // 초기화 작업
  updateStatus("Ready");
}   
void WaypointNavigatorPanel::onLoadCsvClicked()
{
  // CSV 파일에서 좌표 불러오기
  QString file_path = QFileDialog::getOpenFileName(this, "Open CSV File", "", "CSV Files (*.csv)");
  if (!file_path.isEmpty()) {
    if (loadWaypointsFromCsv(file_path)) {
      updateStatus("Loaded waypoints from " + file_path);
    } else {
      updateStatus("Failed to load waypoints.");
    }
  }
}

void WaypointNavigatorPanel::onStartNavigationClicked()
{
  // 네비게이션 시작
  if (waypoints_.empty()) {
    updateStatus("No waypoints loaded.");
    return;
  }
  sendNavigationGoal();
}       

void WaypointNavigatorPanel::onStopNavigationClicked()
{
  // 네비게이션 중지
  cancelNavigationGoal();
  updateStatus("Navigation cancelled.");
}

void WaypointNavigatorPanel::updateStatus(const QString& msg)
{
  if (status_label_) {
    status_label_->setText("Status: " + msg);
  }
}


bool WaypointNavigatorPanel::loadWaypointsFromCsv(const QString& path)
{
  waypoints_.clear();
  csv_path_ = path;
  std::ifstream file(csv_path_.toStdString());
  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open CSV file: %s", csv_path_.toStdString().c_str());
    return false;
  }
  std::string line;
  std::getline(file, line); // Skip header
    while (std::getline(file, line)) 
    {
        std::istringstream ss(line);
        std::string x_str, y_str, z_str;
        if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',') && std::getline(ss, z_str)) {
            geometry_msgs::msg::PoseStamped waypoint;
            waypoint.header.frame_id = "map"; // or your desired frame
            waypoint.pose.position.x = std::stod(x_str);
            waypoint.pose.position.y = std::stod(y_str);
            waypoint.pose.position.z = std::stod(z_str);
            waypoints_.push_back(waypoint);
        }
    }
  file.close();
  RCLCPP_INFO(node_->get_logger(), "Loaded %zu waypoints from %s", waypoints_.size(), csv_path_.toStdString().c_str());
  return true;
}

void WaypointNavigatorPanel::sendNavigationGoal()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    updateStatus("Action server not available.");
    return;
  }

  auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();
  goal_msg.poses = waypoints_;
  
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      updateStatus("Navigation succeeded.");
    } else {
      updateStatus("Navigation failed.");
    }
  };

  action_client_->async_send_goal(goal_msg, send_goal_options);
  updateStatus("Navigation started.");
}
void WaypointNavigatorPanel::cancelNavigationGoal()
{
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_.reset();
    updateStatus("Navigation goal cancelled.");
  } else {
    updateStatus("No active navigation goal to cancel.");
  }
}

} // namespace navi_plugin  

PLUGINLIB_EXPORT_CLASS(
  navi_plugin::WaypointNavigatorPanel,
  rviz_common::Panel
)
