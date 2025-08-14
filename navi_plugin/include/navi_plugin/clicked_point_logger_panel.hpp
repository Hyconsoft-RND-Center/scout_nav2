// include/navi_plugin/clicked_point_logger_panel.hpp
#ifndef NAVI_PLUGIN__CLICKED_POINT_LOGGER_PANEL_HPP_
#define NAVI_PLUGIN__CLICKED_POINT_LOGGER_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include <rclcpp/rclcpp.hpp>
// #include <Q_OBJECT>
#include <QPushButton>
#include <QVBoxLayout>
#include <fstream>
#include <rviz_common/display_context.hpp>
#include <QTimer>
#include <QLabel>
#include <QDesktopServices>
#include <QFileDialog>
#include <QUrl>
#include <QMessageBox>
#include <filesystem>

namespace navi_plugin
{

class ClickedPointLoggerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ClickedPointLoggerPanel(QWidget* parent = nullptr);

  void onInitialize() override;
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

private Q_SLOTS:
  void onOpenCsvClicked();
  void onClearClicked();
  void onDeleteLastClicked();
  void onImportCsvClicked();
  void onSaveClicked();
  bool saveFile(const std::string& file_path);
  void publishWaypoints();
  void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

private:
  // 콜백
  // void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  QTimer* ros_spin_timer_{nullptr};
  
  // RViz가 제공하는 노드를 여기에 저장
  rclcpp::Node::SharedPtr node_;
  // ROS2 subscriber
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  // ROS2 Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_pub_;

  // CSV 파일
  std::string csv_path_{"/tmp/clicked_points.csv"};
  std::ofstream csv_file_;

  // Qt 위젯
  QLabel* current_file_label_{nullptr};
  QLabel* last_point_label_{nullptr};
  QPushButton* import_btn_{nullptr};
  QPushButton* open_current_btn_{nullptr};
  QPushButton* save_btn_{nullptr};
  QPushButton* delete_last_btn_{nullptr};
  QPushButton* clear_btn_{nullptr};

  QMessageBox* ask_clear_{nullptr};

  // 저장된 포인트들
  std::vector<geometry_msgs::msg::Point> points_;
  void save_all_to_csv()
  {
    if (csv_file_.is_open()) {
      csv_file_.seekp(0, std::ios::end); // 파일 끝으로 이동
      for (const auto& pt : points_) {
        csv_file_ << pt.x << "," << pt.y << "," << pt.z << "\n";
      }
      csv_file_.flush(); // 명시적으로 디스크에 기록
    }
  }

protected:
  void makeBackup();
  void restoreFromBackup();
};

}  // namespace navi_plugin

#endif  // NAVI_PLUGIN__CLICKED_POINT_LOGGER_PANEL_HPP_