// follow_waypoints_from_csv_node.cpp
// Build: add_executable(fw_from_csv src/follow_waypoints_from_csv_node.cpp)
//   ament_target_dependencies(fw_from_csv rclcpp rclcpp_action geometry_msgs nav2_msgs tf2 tf2_geometry_msgs)
// Run (Nav2 up): ros2 run <pkg> fw_from_csv --ros-args -p csv:=/path/waypoints.csv -p frame:=map -p spacing:=24.0 -p ensure_first_inside:=true

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "navi_plugin/pose_csv.hpp"

using namespace std::chrono_literals;
namespace nav2 = nav2_msgs::action;

class FollowFromCsvNode : public rclcpp::Node {
public:
  using FW = nav2::FollowWaypoints;
  explicit FollowFromCsvNode() : Node("follow_waypoints_from_csv_node")
  {
    csv_   = this->declare_parameter<std::string>("csv", "");
    frame_ = this->declare_parameter<std::string>("frame", "map");
    spacing_ = this->declare_parameter<double>("spacing", 24.0);
    ensure_first_inside_ = this->declare_parameter<bool>("ensure_first_inside", true);
    bt_xml_ = this->declare_parameter<std::string>("bt", "");

    client_ = rclcpp_action::create_client<FW>(this, "follow_waypoints");
    tf_buf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

    timer_ = this->create_wall_timer(500ms, std::bind(&FollowFromCsvNode::tick, this));
  }

private:
  void tick() {
    if (sent_) return;
    if (csv_.empty()) {
      RCLCPP_ERROR(get_logger(), "Param 'csv' is empty.");
      timer_->cancel();
      return;
    }
    if (!client_->wait_for_action_server(100ms)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /follow_waypoints server...");
      return;
    }

    auto poses = pose_csv::load(csv_, frame_);
    if (poses.empty()) {
      RCLCPP_ERROR(get_logger(), "No waypoints loaded from %s", csv_.c_str());
      timer_->cancel();
      return;
    }

    // Optional: resample to spacing_
    if (spacing_ > 0.0 && poses.size() >= 2) {
      std::vector<geometry_msgs::msg::PoseStamped> res;
      res.reserve(poses.size());
      auto add_between = [&](const geometry_msgs::msg::PoseStamped& a,
                             const geometry_msgs::msg::PoseStamped& b) {
        double dx = b.pose.position.x - a.pose.position.x;
        double dy = b.pose.position.y - a.pose.position.y;
        double dz = b.pose.position.z - a.pose.position.z;
        double L = std::hypot(dx, dy);
        if (L < 1e-6) return;
        int n = static_cast<int>(std::floor(L / spacing_));
        for (int i = 1; i <= n; ++i) {
          double t = (i * spacing_) / L;
          if (t >= 1.0) break;
          geometry_msgs::msg::PoseStamped p = a;
          p.pose.position.x = a.pose.position.x + t * dx;
          p.pose.position.y = a.pose.position.y + t * dy;
          p.pose.position.z = a.pose.position.z + t * dz;
          res.push_back(p);
        }
      };
      res.push_back(poses.front());
      for (size_t i=0; i+1<poses.size(); ++i) {
        add_between(poses[i], poses[i+1]);
        res.push_back(poses[i+1]);
      }
      poses.swap(res);
    }

    // Optional carrot to ensure first is within window distance (approximate)
    if (ensure_first_inside_) {
      try {
        auto tf = tf_buf_->lookupTransform(frame_, "base_link", tf2::TimePointZero, tf2::durationFromSec(0.5));
        double rx = tf.transform.translation.x;
        double ry = tf.transform.translation.y;
        double dx = poses[0].pose.position.x - rx;
        double dy = poses[0].pose.position.y - ry;
        double d = std::hypot(dx, dy);
        if (d > spacing_) {
          geometry_msgs::msg::PoseStamped carrot = poses[0];
          carrot.pose.position.x = rx + spacing_ * dx / d;
          carrot.pose.position.y = ry + spacing_ * dy / d;
          poses.insert(poses.begin(), carrot);
          RCLCPP_INFO(get_logger(), "Inserted carrot waypoint ~%.2fm ahead", spacing_);
        }
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "TF lookup failed (map->base_link): %s", ex.what());
      }
    }

    // Send FollowWaypoints
    auto goal = FW::Goal();
    goal.poses = poses;
    goal.behavior_tree = bt_xml_;
    RCLCPP_INFO(get_logger(), "Sending %zu waypoints from %s", poses.size(), csv_.c_str());
    auto send_goal_options = rclcpp_action::Client<FW>::SendGoalOptions();
    send_goal_options.feedback_callback = [this](auto, const std::shared_ptr<const FW::Feedback> fb) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Current waypoint index: %d", fb->current_waypoint);
    };
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<FW>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "FollowWaypoints succeeded.");
      } else {
        RCLCPP_WARN(get_logger(), "FollowWaypoints finished with code %d", static_cast<int>(result.code));
      }
      if (!result.result->missed_waypoints.empty()) {
        std::string idxs;
        for (auto i : result.result->missed_waypoints) { idxs += std::to_string(i) + " "; }
        RCLCPP_WARN(get_logger(), "Missed waypoint indices: %s", idxs.c_str());
      }
      rclcpp::shutdown();
    };
    client_->async_send_goal(goal, send_goal_options);
    sent_ = true;
  }

  // params
  std::string csv_;
  std::string frame_;
  double spacing_;
  bool ensure_first_inside_;
  std::string bt_xml_;

  // infra
  rclcpp_action::Client<FW>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  bool sent_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowFromCsvNode>());
  rclcpp::shutdown();
  return 0;
}