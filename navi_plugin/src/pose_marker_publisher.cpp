#include "pose_marker_publisher/pose_marker_publisher.hpp"



using namespace pose_marker_publisher;

PoseMarkerPublisher::PoseMarkerPublisher()
: Node("pose_marker_publisher")
{
  sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    "/wayposes", 10,
    std::bind(&PoseMarkerPublisher::posesCallback, this, std::placeholders::_1));
  pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/wayposes_marker", 10);
}

void PoseMarkerPublisher::posesCallback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  // 예: msg->header.frame_id == "import" 이면 full_replace 모드로 처리
  bool full_replace = (msg->header.frame_id == "import");
  updateMarkers(msg->header, msg->poses, full_replace);
}

void PoseMarkerPublisher::updateMarkers(
  const std_msgs::msg::Header& h,
  const std::vector<geometry_msgs::msg::Pose>& new_poses,
  bool full_replace)
{
  if (full_replace) {
    // 1) 전체 삭제
    clearMarkers(prev_poses_.size());
    // 2) 전체 추가
    publishAllMarkers(h, new_poses);
    // 3) 상태 동기화
    prev_poses_ = new_poses;
    return;
  }

  // 증분(diff) 업데이트
  updateDiff(h, prev_poses_, new_poses);
  prev_poses_ = new_poses;
}

void PoseMarkerPublisher::clearMarkers(size_t count)
{
  std_msgs::msg::Header hdr;
  hdr.stamp    = now();
  hdr.frame_id = "map";  // 또는 파라미터로 받은 frame

  visualization_msgs::msg::MarkerArray arr;
  for (size_t i = 0; i < count; ++i) {
    arr.markers.push_back(makeDeleteMarker(hdr, "waypoint_arrow",  i));
    arr.markers.push_back(makeDeleteMarker(hdr, "waypoint_sphere",1000+i));
    arr.markers.push_back(makeDeleteMarker(hdr, "waypoint_text",  2000+i));
  }
  // LineStrip 삭제
  arr.markers.push_back(makeDeleteMarker(hdr, "waypoint_line", 3000));

  pub_->publish(arr);
}

void PoseMarkerPublisher::publishAllMarkers(
  const std_msgs::msg::Header& h,
  const std::vector<geometry_msgs::msg::Pose>& poses)
{
  visualization_msgs::msg::MarkerArray arr;
  for (size_t i = 0; i < poses.size(); ++i) {
    arr.markers.push_back(makeArrow(h, poses[i], i));
    arr.markers.push_back(makeSphere(h, poses[i], i+1000));
    arr.markers.push_back(makeText(h, poses[i], i+2000));
  }
  if (poses.size()>1) {
    arr.markers.push_back(makeLineStrip(h, poses));
  }
  pub_->publish(arr);
}

void PoseMarkerPublisher::updateDiff(
  const std_msgs::msg::Header& hdr,
  const std::vector<geometry_msgs::msg::Pose>& oldv,
  const std::vector<geometry_msgs::msg::Pose>& newv)
{
  visualization_msgs::msg::MarkerArray arr;

  for (size_t i = 0; i < oldv.size(); ++i) {
    bool changed = (i >= newv.size() || !posesEqual(oldv[i], newv[i]));
    if (!changed) continue;
    arr.markers.push_back(makeDeleteMarker(hdr, "waypoint_arrow",  i));
    arr.markers.push_back(makeDeleteMarker(hdr, "waypoint_sphere",1000+i));
    arr.markers.push_back(makeDeleteMarker(hdr, "waypoint_text",  2000+i));
  }

  for (size_t i = 0; i < newv.size(); ++i) {
    bool changed = (i >= oldv.size() || !posesEqual(oldv[i], newv[i]));
    if (!changed) continue;
    arr.markers.push_back(makeArrow(hdr, newv[i], i));
    arr.markers.push_back(makeSphere(hdr, newv[i], 1000+i));
    arr.markers.push_back(makeText(hdr, newv[i], 2000+i));
  }

  // LineStrip 은 매 번 갱신
  if (newv.size()>1) {
    arr.markers.push_back(makeDeleteMarker(hdr, "waypoint_line", 3000));
    arr.markers.push_back(makeLineStrip(hdr, newv));
  }

  pub_->publish(arr);
}

std::tuple<double,double,double> PoseMarkerPublisher::extractRPY(
  const geometry_msgs::msg::Pose & p)
{
  tf2::Quaternion q;
  tf2::fromMsg(p.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}

// Arrow marker
visualization_msgs::msg::Marker PoseMarkerPublisher::makeArrow(
  const std_msgs::msg::Header & header,
  const geometry_msgs::msg::Pose & pose,
  size_t id)
{
  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns     = "waypoint_arrow";
  m.id     = id;
  m.type   = m.ARROW;
  m.action = m.ADD;
  m.pose   = pose;
  m.scale.x = 0.3; m.scale.y = 0.07; m.scale.z = 0.07;
  m.color.r = 1.0; m.color.g = 0.3; m.color.b = 0.0; m.color.a = 1.0;
  return m;
}

// Sphere marker
visualization_msgs::msg::Marker PoseMarkerPublisher::makeSphere(
  const std_msgs::msg::Header & header,
  const geometry_msgs::msg::Pose & pose,
  size_t id)
{
  auto m = makeArrow(header, pose, id);
  m.ns   = "waypoint_sphere";
  m.type = m.SPHERE;
  return m;
}

// Text marker
visualization_msgs::msg::Marker PoseMarkerPublisher::makeText(
  const std_msgs::msg::Header & header,
  const geometry_msgs::msg::Pose & pose,
  size_t id)
{
  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns     = "waypoint_text";
  m.id     = id;
  m.type   = m.TEXT_VIEW_FACING;
  m.action = m.ADD;
  m.pose   = pose;
  m.pose.position.z += 0.22;
  m.scale.z = 0.13;
  m.color.r = m.color.g = m.color.b = 1.0; m.color.a = 1.0;
  m.text    = std::to_string(id % 1000);
  return m;
}

// Line strip marker
visualization_msgs::msg::Marker PoseMarkerPublisher::makeLineStrip(
  const std_msgs::msg::Header & header,
  const std::vector<geometry_msgs::msg::Pose> & poses)
{
  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns     = "waypoint_line";
  m.id     = 3000;
  m.type   = m.LINE_STRIP;
  m.action = m.ADD;
  m.scale.x = 0.03;
  m.color.b = 1.0; m.color.a = 1.0;
  for (auto & p : poses) {
    m.points.push_back(p.position);
  }
  return m;
}

bool PoseMarkerPublisher::posesEqual(
  const geometry_msgs::msg::Pose & a,
  const geometry_msgs::msg::Pose & b)
{
  // 위치 비교
  constexpr double eps = 1e-6;
  if (std::abs(a.position.x - b.position.x) > eps) return false;
  if (std::abs(a.position.y - b.position.y) > eps) return false;
  if (std::abs(a.position.z - b.position.z) > eps) return false;

  // 회전(쿼터니언) 비교(or RPY 비교)
  tf2::Quaternion qa, qb;
  tf2::fromMsg(a.orientation, qa);
  tf2::fromMsg(b.orientation, qb);
  return qa.angleShortestPath(qb) < eps;
}

// 2) makeDeleteMarker 구현 (id 타입 일치 주의: header에 선언된 그대로)
visualization_msgs::msg::Marker PoseMarkerPublisher::makeDeleteMarker(
  const std_msgs::msg::Header & hdr,
  const std::string & ns,
  size_t id)  // <- 이 타입이 헤더와 동일해야 합니다
{
  visualization_msgs::msg::Marker m;
  m.header = hdr;
  m.ns     = ns;
  m.id     = static_cast<int>(id); // Marker.id는 int니까 캐스팅도 해 줄 수 있고
  m.action = m.DELETE;
  return m;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}