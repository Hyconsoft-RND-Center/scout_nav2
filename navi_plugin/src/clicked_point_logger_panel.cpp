// src/clicked_point_logger_panel.cpp
#include "navi_plugin/clicked_point_logger_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <filesystem>

namespace navi_plugin
{
ClickedPointLoggerPanel::ClickedPointLoggerPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  node_ = std::make_shared<rclcpp::Node>("clicked_point_logger_panel_node");

  // 마지막 포인트 레이블
  last_point_label_ = new QLabel("Last: (none)", this);
  current_file_label_ = new QLabel(QString("Current File: %1").arg(QString::fromStdString(csv_path_)), this);
  

  // 버튼 레이아웃
  save_btn_ = new QPushButton("Save Points", this);
  connect(save_btn_, &QPushButton::clicked, this, &ClickedPointLoggerPanel::onSaveClicked);
  import_btn_ = new QPushButton("Import Points", this);
  connect(import_btn_, &QPushButton::clicked, this, &ClickedPointLoggerPanel::onImportCsvClicked);
  open_current_btn_ = new QPushButton("Open CSV", this);
  connect(open_current_btn_, &QPushButton::clicked, this, &ClickedPointLoggerPanel::onOpenCsvClicked);
  delete_last_btn_ = new QPushButton("Delete Last Point", this);
  connect(delete_last_btn_, &QPushButton::clicked, this, &ClickedPointLoggerPanel::onDeleteLastClicked);
  clear_btn_ = new QPushButton("Clear All Points", this);
  connect(clear_btn_, &QPushButton::clicked, this, &ClickedPointLoggerPanel::onClearClicked);
  
  auto* main_layout_ = new QVBoxLayout;
  main_layout_->addWidget(last_point_label_);
  main_layout_->addWidget(current_file_label_);
  
  auto* row_= new QHBoxLayout;
  row_->addWidget(save_btn_);
  row_->addWidget(import_btn_);
  row_->addWidget(open_current_btn_);
  main_layout_->addWidget(delete_last_btn_);
  main_layout_->addWidget(clear_btn_);

  // 레이아웃 설정
  main_layout_->addLayout(row_);
  // 버튼 레이아웃 생성
  setLayout(main_layout_);
}

void ClickedPointLoggerPanel::onInitialize()
{
  // CSV 초기화
  csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
  if (csv_file_) {
    csv_file_ << "x,y,z\n";
    csv_file_.flush();
  }

  current_file_label_->setText(QString("Current File: %1").arg(QString::fromStdString(csv_path_)));

  sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&ClickedPointLoggerPanel::point_callback, this, std::placeholders::_1));
  waypoint_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);

  ros_spin_timer_ = new QTimer(this);
  connect(ros_spin_timer_, &QTimer::timeout, [this]() {
      rclcpp::spin_some(node_);
  });
  ros_spin_timer_->start(100);  // 0.1s마다 spin_some 호출
}


void ClickedPointLoggerPanel::point_callback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  if (!csv_file_.is_open()){
        csv_file_.open(csv_path_, std::ios::app);
        if (!csv_file_) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
            return;
        }
  }
  
  points_.push_back(msg->point);
    if (last_point_label_) {
        last_point_label_->setText(
        QString("Last: (%1, %2, %3)")
        .arg(msg->point.x, 0, 'f', 2)
        .arg(msg->point.y, 0, 'f', 2)
        .arg(msg->point.z, 0, 'f', 2)
        );
    }

    // point 좌표를 한 줄로 기록
    if (csv_file_.is_open()) {
        csv_file_ << msg->point.x << ","
                  << msg->point.y << ","
                  << msg->point.z << std::endl;
        RCLCPP_INFO(node_->get_logger(), "Saved point: (%.2f, %.2f, %.2f)",
                    msg->point.x, msg->point.y, msg->point.z);
        // 파일을 디스크에 기록 
        csv_file_.flush();
    }
    publishWaypoints();
}

void ClickedPointLoggerPanel::onOpenCsvClicked()
{
  QDesktopServices::openUrl(QUrl::fromLocalFile(QString::fromStdString(csv_path_)));
}
void ClickedPointLoggerPanel::onClearClicked() {
    makeBackup();
    auto ask_clear_ = QMessageBox::question(
        this,
        tr("Clear ALL Points"), // Title
        tr("You cannot revert after clearing all points.\nDo you want to continue?"), // Text
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No // 기본선택: No
    );
    if (ask_clear_ == QMessageBox::Yes) {
        points_.clear();
        save_all_to_csv();
        if (last_point_label_) {
            last_point_label_->setText("Last: (none)");
        }
    }
    publishWaypoints(); // 포인트 삭제 후 퍼블리시
}

void ClickedPointLoggerPanel::onDeleteLastClicked() {
    if (!points_.empty()) {
        points_.pop_back();
        publishWaypoints(); // 마지막 포인트 삭제 후 퍼블리시
        save_all_to_csv();
        if (!points_.empty()) {
            const auto& pt = points_.back();
            last_point_label_->setText(QString("Last: (%.2f, %.2f, %.2f)").arg(pt.x).arg(pt.y).arg(pt.z));
        } else {
            last_point_label_->setText("Last: (none)");
        }
    }
}
void ClickedPointLoggerPanel::onImportCsvClicked()
{
  QString file_path = QFileDialog::getOpenFileName(this, "Import CSV File", QString::fromStdString(csv_path_), "CSV Files (*.csv)");
  if (!file_path.isEmpty()) {
    csv_path_ = file_path.toStdString();
    current_file_label_->setText(QString("Current File: %1").arg(file_path));
    csv_file_.close();
    csv_file_.open(csv_path_, std::ios::out | std::ios::app);
    if (!csv_file_) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
    }
  }
  publishWaypoints(); // CSV 파일 변경 후 퍼블리시
}
void ClickedPointLoggerPanel::onSaveClicked()
{
    // 1. 파일 저장 다이얼로그
    QString suggested = QString::fromStdString(csv_path_);
    QString fileName = QFileDialog::getSaveFileName(
        this, tr("Save Points"), suggested, tr("CSV Files (*.csv)")
    );
    if (fileName.isEmpty()) {
        // 사용자 취소
        return;
    }

    bool ok = saveFile(fileName.toStdString());
    if (ok) {
        QMessageBox::information(this, tr("Save Successful"),
            tr("Saved waypoints to %1.").arg(fileName));
        RCLCPP_INFO(node_->get_logger(), "Saved points to %s", fileName.toStdString().c_str());
        // 라벨 등 UI 경로 동기화
        // current_file_label_->setText(fileName);
    } else {
        QMessageBox::critical(this, tr("Failed to Save"),
            tr("Failed to save!\nCheck permission or directory."));
    }
}
bool ClickedPointLoggerPanel::saveFile(const std::string& file_path)
{
    makeBackup();
    std::ofstream out_file(file_path, std::ios::out | std::ios::trunc);
    if (!out_file) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open file for writing: %s", file_path.c_str());
        return false;
    }
    out_file << "x,y,z\n";  // CSV 헤더
    for (const auto& pt : points_) {
        out_file << pt.x << "," << pt.y << "," << pt.z << "\n";
    }
    out_file.close();
    if (!out_file) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to write to file: %s", file_path.c_str());
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Successfully saved points to %s", file_path.c_str());
    return true;    
}

void ClickedPointLoggerPanel::makeBackup()
{
    std::string backup_path = csv_path_ + ".backup";
    if (std::filesystem::exists(csv_path_)) {
        std::filesystem::copy(csv_path_, backup_path, std::filesystem::copy_options::overwrite_existing);
    }
}

void ClickedPointLoggerPanel::restoreFromBackup()
{
    std::string backup_path = csv_path_ + ".backup";
    if (std::filesystem::exists(backup_path)) {
        std::filesystem::copy(backup_path, csv_path_, std::filesystem::copy_options::overwrite_existing);
        // backup 복원 후 points_를 다시 로드할 필요가 있으면 이곳에서 호출!
        // 예: loadPointsFromFile(csv_path_);
    }
}
void ClickedPointLoggerPanel::publishWaypoints()
{
    if (!waypoint_pub_) return;
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "map"; // 필요시 frame_id 맞춰서
    for (const auto& pt : points_) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = pt.x;
        pose.position.y = pt.y;
        pose.position.z = pt.z;
        pose.orientation.w = 1.0; // 단순 포인트라면 orientation 기본값
        msg.poses.push_back(pose);
    }
    waypoint_pub_->publish(msg);
}

}  // namespace navi_plugin

PLUGINLIB_EXPORT_CLASS(
  navi_plugin::ClickedPointLoggerPanel,
  rviz_common::Panel
)