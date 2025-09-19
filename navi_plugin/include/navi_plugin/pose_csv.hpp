#pragma once
// pose_csv.hpp -- minimal CSV loader/saver for geometry_msgs::msg::PoseStamped (x,y,z,roll,pitch,yaw)
// Format:
//   Header (optional): x,y,z,roll,pitch,yaw
//   Rows: x,y,z,roll,pitch,yaw  (angles in radians)
// Loads into PoseStamped with header.frame_id provided by caller.

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cctype>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pose_csv {

inline bool is_header_line(const std::string& line) {
  // treat as header if any alphabetic char exists
  for (unsigned char c : line) {
    if (std::isalpha(c)) return true;
  }
  return false;
}

inline std::vector<geometry_msgs::msg::PoseStamped>
load(const std::string& path, const std::string& frame_id)
{
  std::vector<geometry_msgs::msg::PoseStamped> out;
  std::ifstream in(path);
  if (!in) return out;
  std::string line;
  if (std::getline(in, line)) {
    if (!line.empty() && !is_header_line(line)) {
      // first line is data; process it below by rewinding the string
      std::stringstream ss(line);
      std::array<double,6> v;
      for (int i=0;i<6;i++) {
        std::string tok; if (!std::getline(ss, tok, ',')) break;
        v[i] = std::stod(tok);
      }
      if (ss || ss.eof()) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id;
        tf2::Quaternion q; q.setRPY(v[3], v[4], v[5]); q.normalize();
        ps.pose.position.x = v[0];
        ps.pose.position.y = v[1];
        ps.pose.position.z = v[2];
        ps.pose.orientation = tf2::toMsg(q);
        out.push_back(ps);
      }
    }
  }
  // Continue with remaining lines
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::array<double,6> v;
    for (int i=0;i<6;i++) {
      std::string tok; if (!std::getline(ss, tok, ',')) break;
      v[i] = std::stod(tok);
    }
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id;
    tf2::Quaternion q; q.setRPY(v[3], v[4], v[5]); q.normalize();
    ps.pose.position.x = v[0];
    ps.pose.position.y = v[1];
    ps.pose.position.z = v[2];
    ps.pose.orientation = tf2::toMsg(q);
    out.push_back(ps);
  }
  return out;
}

inline bool save(const std::string& path,
                 const std::vector<geometry_msgs::msg::PoseStamped>& poses)
{
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) return false;
  out << "x,y,z,roll,pitch,yaw\n";
  for (const auto& ps : poses) {
    const auto& p = ps.pose;
    tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    double r, pt, yw;
    tf2::Matrix3x3(q).getRPY(r, pt, yw);
    out << p.position.x << ","
        << p.position.y << ","
        << p.position.z << ","
        << r << "," << pt << "," << yw << "\n";
  }
  return true;
}

} // namespace pose_csv