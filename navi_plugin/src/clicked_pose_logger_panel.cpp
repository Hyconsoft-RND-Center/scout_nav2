// clicked_pose_logger_panel.cpp
#include "navi_plugin/clicked_pose_logger_panel.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace navi_plugin
{
namespace {
  std::tuple<double,double,double> extractRPY(const geometry_msgs::msg::Pose & p) {
    tf2::Quaternion q;
    tf2::fromMsg(p.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
  }
}

ClickedPoseLoggerPanel::ClickedPoseLoggerPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
    node_ = std::make_shared<rclcpp::Node>("clicked_pose_logger_panel__node");
    // UI 초기화
    last_point_label_ = new QLabel("Last: (none)", this);
    current_file_label_ = new QLabel(QString("Current File: %1").arg(QString::fromStdString(csv_path_)), this); 
    
    import_btn_ = new QPushButton("Import CSV", this);
    open_current_btn_ = new QPushButton("Open Current CSV", this);
    save_btn_ = new QPushButton("Save CSV", this);
    delete_last_btn_ = new QPushButton("Delete Last Pose", this);
    clear_btn_ = new QPushButton("Clear All Poses", this);  
    preview_btn_ = new QPushButton("Preview Path", this);
    clear_path_btn_ = new QPushButton("Clear Preview Path", this);
    
    connect(import_btn_, &QPushButton::clicked, this, &ClickedPoseLoggerPanel::onImportCsvClicked);
    connect(open_current_btn_, &QPushButton::clicked, this, &ClickedPoseLoggerPanel::onOpenCsvClicked);
    connect(save_btn_, &QPushButton::clicked, this, &ClickedPoseLoggerPanel::onSaveClicked);
    connect(delete_last_btn_, &QPushButton::clicked, this, &ClickedPoseLoggerPanel::onDeleteLastClicked);
    connect(clear_btn_, &QPushButton::clicked, this, &ClickedPoseLoggerPanel::onClearClicked);  
    connect(preview_btn_, &QPushButton::clicked, this, &ClickedPoseLoggerPanel::onPreviewClicked);
    connect(clear_path_btn_, &QPushButton::clicked, this, &ClickedPoseLoggerPanel::onClearPathClicked);

    auto* main_layout_ = new QVBoxLayout;
    main_layout_->addWidget(last_point_label_);
    main_layout_->addWidget(current_file_label_);
    auto* row_ = new QHBoxLayout;
    row_->addWidget(import_btn_);
    row_->addWidget(open_current_btn_);
    row_->addWidget(save_btn_);
    main_layout_->addLayout(row_);
    auto* row2_ = new QHBoxLayout;
    row2_->addWidget(clear_btn_);
    row2_->addWidget(delete_last_btn_);
    main_layout_->addLayout(row2_);
    auto* row3_ = new QHBoxLayout;
    row3_->addWidget(clear_path_btn_);
    row3_->addWidget(preview_btn_);
    main_layout_->addLayout(row3_);

    setLayout(main_layout_);
}

void ClickedPoseLoggerPanel::onInitialize()
{
  // CSV 파일 오픈 & 헤더 쓰기
  csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (csv_file_.is_open()) {
    csv_file_ << "x,y,z,roll,pitch,yaw\n";
    csv_file_.flush();
    }
  current_file_label_->setText(
    QString("Current File: %1").arg(QString::fromStdString(csv_path_)));

  // PoseStamped 구독
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/clicked_pose2d", 10,
    std::bind(&ClickedPoseLoggerPanel::poseCallback, this, std::placeholders::_1)
  );

  // PoseArray 퍼블리셔
  waypoint_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
    "/wayposes", 10);

  // ComputePathThroughPoses 액션 클라이언트
  planner_client_ = rclcpp_action::create_client<ComputePathThroughPoses>(
      node_, "/compute_path_through_poses");

  // Path 퍼블리셔
  rclcpp::QoS viz_qos(1);
  viz_qos.transient_local().reliable();
  preview_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "/preview_path", viz_qos);

  // Qt 타이머로 spin_some()
  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, [this]() {
    rclcpp::spin_some(node_);
  });
  ros_spin_timer_->start(1000); // 1초마다 spin_some() 호출 = 1초 마다 rviz2 패널 업데이트
}
void ClickedPoseLoggerPanel::poseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // orientation → roll, pitch, yaw
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // PoseStamped 
  geometry_msgs::msg::PoseStamped ps = *msg;
  if (ps.header.frame_id.empty()) {
    ps.header.frame_id = fixed_frame_; // 기본 프레임 ID 설정
  }
  if (rclcpp::Time(ps.header.stamp).nanoseconds() == 0) {
    ps.header.stamp = node_->now(); // 현재 시간으로 설정
  }

  // Quternion Standardization
  {
    tf2::Quaternion q; tf2::fromMsg(ps.pose.orientation, q);
    if (q.length2() == 0) q.setRPY(0,0,0);
    q.normalize();
    ps.pose.orientation = tf2::toMsg(q);
  }

  poses_.push_back(ps);
  publishPoses();
  updateLabel();
  flushCSV(); 
}
bool ClickedPoseLoggerPanel::saveFile(const std::string& file_path)
{
  std::ofstream out(file_path, std::ios::out | std::ios::trunc);
  if (!out) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot open %s for saving", file_path.c_str());
    return false;
  }

  // 6컬럼 헤더
  out << "x,y,z,roll,pitch,yaw\n";
  for (const auto& ps : poses_) {
    const auto& p = ps.pose;
    // Quaternion → RPY 변환
    // tf2::Quaternion q = tf2::fromMsg(p.orientation);
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // CSV 포맷으로 출력
    tf2::Quaternion q(
      p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    double r, pt, yw;
    tf2::Matrix3x3(q).getRPY(r, pt, yw);
    out
      << p.position.x << "," 
      << p.position.y << "," 
      << p.position.z << ","
      << r            << "," 
      << pt           << "," 
      << yw           << "\n";
  }
  return true;
}

bool ClickedPoseLoggerPanel::loadFile(const std::string& file_path)
{
  std::ifstream in(file_path);
  if (!in) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot open %s for loading", file_path.c_str());
    return false;
  }

  poses_.clear();
  std::string line;
  std::getline(in, line);  // 헤더 스킵

  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::array<double,6> v;
    for (int i = 0; i < 6; ++i) {
      std::string tok;
      if (!std::getline(ss, tok, ',')) break;
      v[i] = std::stod(tok);
    }
    addPose(v[0], v[1], v[2], v[3], v[4], v[5]);
  }

  // 불러온 뒤 퍼블리시 & 라벨 업데이트
  publishPoses();
  updateLabel();
  clearPreviewPath();
  return true;
}

void ClickedPoseLoggerPanel::flushCSV(){
  if (!csv_file_.is_open()) return;
  std::ofstream out(csv_path_, std::ios::out | std::ios::app);
  if (!out) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to flush CSV file");
    return;
  }
  const auto& p = poses_.back().pose;
  auto [roll, pitch, yaw] = extractRPY(p);
  out << p.position.x << ',' << p.position.y << ',' << p.position.z << ','
      << roll      << ',' << pitch          << ',' << yaw << '\n';
  out.flush();
}

void ClickedPoseLoggerPanel::addPose(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = fixed_frame_;
  ps.header.stamp = node_->now();
  tf2::Quaternion qq; qq.setRPY(roll, pitch, yaw); qq.normalize();
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = z;
  ps.pose.orientation = tf2::toMsg(qq);
  poses_.push_back(ps);
  publishPoses();
  updateLabel();
  flushCSV();
}

void ClickedPoseLoggerPanel::saveAllPosesToCSV()
{
  if (!csv_file_.is_open()) return;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3);

  for (const auto& ps : poses_) {
    const auto& p = ps.pose;
    auto [roll, pitch, yaw] = extractRPY(p);
    oss << p.position.x << ',' << p.position.y << ',' << p.position.z << ','
        << roll      << ',' << pitch          << ',' << yaw << '\n';
  }

  csv_file_ << oss.str();
  if (!csv_file_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed writing poses to CSV");
  }
  csv_file_.flush();
}
void ClickedPoseLoggerPanel::updateLabel()
{
  // 1) 데이터가 없으면 “none” 처리
  if (poses_.empty()) {
    last_point_label_->setText("Last: (none)");
    return;
  }

  // 2) 가장 마지막 포즈 꺼내기
  const auto& p = poses_.back().pose;

  // 3) quaternion → roll, pitch, yaw 계산 (radian)
  tf2::Quaternion q;
  tf2::fromMsg(p.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // 4) 라벨에 텍스트 세팅 (2자리 소수, 각도를 degree 단위로 보고 싶으면 *180/M_PI)
  last_point_label_->setText(
    QString("Last: x=%1, y=%2, z=%3 | r=%4°, p=%5°, y=%6°")
      .arg(p.position.x, 0, 'f', 2)
      .arg(p.position.y, 0, 'f', 2)
      .arg(p.position.z, 0, 'f', 2)
      .arg(roll   * 180.0 / M_PI, 0, 'f', 1)
      .arg(pitch  * 180.0 / M_PI, 0, 'f', 1)
      .arg(yaw    * 180.0 / M_PI, 0, 'f', 1)
  );
}
void ClickedPoseLoggerPanel::onOpenCsvClicked()
{
  QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(csv_path_)));
}
void ClickedPoseLoggerPanel::onClearClicked()
{
  if (!ask_clear_) {
    ask_clear_ = new QMessageBox(QMessageBox::Question, "Clear Poses",
      "Are you sure you want to clear all poses?",
      QMessageBox::Yes | QMessageBox::No, this);
    ask_clear_->setDefaultButton(QMessageBox::No);      
    connect(ask_clear_, &QMessageBox::finished, this, [this](int result) {
      if (result == QMessageBox::Yes) {
        poses_.clear();
        publishPoses();
        updateLabel();
        if (csv_file_.is_open()) {
          csv_file_.close();
          csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
          csv_file_ << "x,y,z,roll,pitch,yaw\n";
          csv_file_.flush();
        }
      }
    });
    }
    ask_clear_->exec();
}
void ClickedPoseLoggerPanel::onImportCsvClicked()
{
  QString file_path = QFileDialog::getOpenFileName(
    this,
    "Import CSV File",
    QString::fromStdString(csv_path_),
    "CSV Files (*.csv)");

  if (file_path.isEmpty()) {
    return;
  }

  csv_path_ = file_path.toStdString();
  current_file_label_->setText(
    QString("Current File: %1").arg(file_path));

  csv_file_.close();
  csv_file_.open(csv_path_, std::ios::out | std::ios::app);
  if (!csv_file_) {
    RCLCPP_ERROR(node_->get_logger(),
      "Failed to open CSV file: %s", csv_path_.c_str());
    return;
  }

  if (loadFile(csv_path_)) {
    publishPoses();
    updateLabel();
    RCLCPP_INFO(node_->get_logger(),
      "Imported poses from %s", csv_path_.c_str());
  }
}

void ClickedPoseLoggerPanel::onSaveClicked()
{
  QString suggested = QString::fromStdString(csv_path_);
  QString fileName = QFileDialog::getSaveFileName(
      this,
      tr("Save Poses"),
      suggested,
      tr("CSV Files (*.csv)")
  );

  if (fileName.isEmpty()) {
    return;
  }

  csv_path_ = fileName.toStdString();
  current_file_label_->setText(
    QString("Current File: %1").arg(fileName)
  );

  csv_file_.close();
  csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);

  if (!csv_file_) {
    QMessageBox::critical(
      this,
      tr("Failed to Save"),
      tr("Cannot open file for saving.\nCheck permission or directory.")
    );
    RCLCPP_ERROR(node_->get_logger(),
      "Failed to open CSV file: %s", csv_path_.c_str());
    return;
  }

  bool ok = saveFile(csv_path_);
  if (ok) {
    QMessageBox::information(
      this,
      tr("Save Successful"),
      tr("Saved poses to %1.").arg(fileName)
    );
    RCLCPP_INFO(node_->get_logger(),
      "Saved poses to %s", csv_path_.c_str());
  } else {
    QMessageBox::critical(
      this,
      tr("Failed to Save"),
      tr("Failed to write to CSV file.")
    );
    RCLCPP_ERROR(node_->get_logger(),
      "Failed to save poses to %s", csv_path_.c_str());
  }
}

void ClickedPoseLoggerPanel::onDeleteLastClicked()
{
  if (poses_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No poses to delete");
    return;
  }
  poses_.pop_back();
  publishPoses();
  updateLabel(); 
    if (csv_file_.is_open()) {
        csv_file_.seekp(0, std::ios::end);
        csv_file_.seekp(-1, std::ios::cur); // 마지막 줄 제거
        csv_file_.put('\n'); // 새 줄 추가
        csv_file_.flush();
    }   
    RCLCPP_INFO(node_->get_logger(), "Deleted last pose");
  clearPreviewPath(); 
}
void ClickedPoseLoggerPanel::clearPreviewPath()
{
   if (!preview_pub_) return;
  nav_msgs::msg::Path empty;
  empty.header.frame_id = fixed_frame_;      
  empty.header.stamp    = node_->now();
  preview_pub_->publish(empty);                  // latched(=transient_local) 갱신
  RCLCPP_INFO(node_->get_logger(), "Preview path cleared.");
}

void ClickedPoseLoggerPanel::publishPoses()
{
    if (!waypoint_pub_) return;
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = fixed_frame_;  // 적절한 프레임 ID 설정
    msg.header.stamp = node_->now();
    msg.poses.resize(poses_.size());  // 벡터의 Pose들을 그대로 할당
    for (size_t i = 0; i < poses_.size(); ++i) {
        msg.poses[i] = poses_[i].pose;  // PoseArray에 Pose들을 복사
    }
    // 퍼블리시
    waypoint_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Published %zu poses", poses_.size()); 
}

void ClickedPoseLoggerPanel::onPreviewClicked() {
  if (poses_.size() < 2 && !use_explicit_start_) {
    QMessageBox::warning(this, "Plan", "Need at least 2 poses (start + goal) or set explicit start.");
    return;
  }
  if (!planner_client_ || !planner_client_->wait_for_action_server(std::chrono::seconds(2))) {
    QMessageBox::warning(this, "Plan", "Planner action server not available.");
    return;
  }

  const auto now = node_->now();

  ComputePathThroughPoses::Goal g;

  if (use_explicit_start_) {
    g.use_start = true;
    g.start = explicit_start_;
    if (g.start.header.frame_id.empty()) {
      g.start.header.frame_id = fixed_frame_;
    }
    if (rclcpp::Time(g.start.header.stamp).nanoseconds() == 0) {
      g.start.header.stamp = now;
    }
    g.goals = poses_;
  } else {
    if (poses_.size() < 2) {
      QMessageBox::warning(this, "Plan", "Need at least 2 poses (start + goal).");
      return;
    }
    g.use_start = true;
    g.start = poses_.front();
    g.goals.assign(poses_.begin() + 1, poses_.end());
  }

  g.planner_id = "";
  auto opts = rclcpp_action::Client<ComputePathThroughPoses>::SendGoalOptions{};
  opts.result_callback = [this](const GoalHandleCPTP::WrappedResult & res){
    if (res.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(node_->get_logger(), "Planner failed.");
      return;
    }
    auto path = res.result->path;
    if (path.header.frame_id.empty()) path.header.frame_id = fixed_frame_;
    path.header.stamp = node_->now();
    if (preview_pub_) preview_pub_->publish(path);
    RCLCPP_INFO(node_->get_logger(), "Preview path published: %zu poses", path.poses.size());
  };

  planner_client_->async_send_goal(g, opts);
}
void ClickedPoseLoggerPanel::onClearPathClicked()
{
  clearPreviewPath();
  RCLCPP_INFO(node_->get_logger(), "Preview path cleared.");  
}

}// namespace navi_plugin
 

PLUGINLIB_EXPORT_CLASS(
  navi_plugin::ClickedPoseLoggerPanel,
  rviz_common::Panel
)