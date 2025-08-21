// pose_marker_publisher/pose_marker_publisher.hpp

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>
#include <tuple>

namespace pose_marker_publisher
{

class PoseMarkerPublisher : public rclcpp::Node
{
public:
  PoseMarkerPublisher();

private:
  // rclcpp 콜백
  void posesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // 전체 교체 vs 증분 업데이트 공통 함수
  void updateMarkers(
    const std_msgs::msg::Header& header,
    const std::vector<geometry_msgs::msg::Pose>& new_poses,
    bool full_replace);

  // 전체 삭제
  void clearMarkers(size_t count);

  // 전체 추가
  void publishAllMarkers(
    const std_msgs::msg::Header& header,
    const std::vector<geometry_msgs::msg::Pose>& poses);

  // 증분 추가·삭제
  void updateDiff(
    const std_msgs::msg::Header& header,
    const std::vector<geometry_msgs::msg::Pose>& old_poses,
    const std::vector<geometry_msgs::msg::Pose>& new_poses);

  // 마커 팩토리
  visualization_msgs::msg::Marker makeDeleteMarker(
    const std_msgs::msg::Header& header,
    const std::string& ns, size_t id);
  visualization_msgs::msg::Marker makeArrow(
    const std_msgs::msg::Header& h,
    const geometry_msgs::msg::Pose& p,
    size_t id);
  visualization_msgs::msg::Marker makeSphere(
    const std_msgs::msg::Header& h,
    const geometry_msgs::msg::Pose& p,
    size_t id);
  visualization_msgs::msg::Marker makeText(
    const std_msgs::msg::Header& h,
    const geometry_msgs::msg::Pose& p,
    size_t id);
  
  // 헬퍼: quaternion → RPY, pose 비교
  std::tuple<double,double,double> extractRPY(
    const geometry_msgs::msg::Pose& p);
  bool posesEqual(
    const geometry_msgs::msg::Pose& a,
    const geometry_msgs::msg::Pose& b);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

  // 이전 상태 보관
  std::vector<geometry_msgs::msg::Pose> prev_poses_;
};

} // namespace pose_marker_publisher

