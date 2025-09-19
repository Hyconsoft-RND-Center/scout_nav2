#ifndef RECOVERY_DECISION_NODE_HPP_
#define RECOVERY_DECISION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "recovery_interfaces/srv/get_recovery_maneuver.hpp" 

#include <map>
#include <string>

class RecoveryDecisionNode : public rclcpp::Node
{
public:
  explicit RecoveryDecisionNode(const rclcpp::NodeOptions & options);

private:
  // 멤버 변수
  rclcpp::Service<recovery_interfaces::srv::GetRecoveryManeuver>::SharedPtr service_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_frontal_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_left_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_right_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_rear_;
  
  std::map<std::string, bool> obstacle_status_;

  // 콜백 함수
  void frontal_obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void left_obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void right_obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void rear_obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg);

  void get_recovery_maneuver_callback(
    const std::shared_ptr<recovery_interfaces::srv::GetRecoveryManeuver::Request> request,
    std::shared_ptr<recovery_interfaces::srv::GetRecoveryManeuver::Response>      response);
};

#endif // RECOVERY_DECISION_NODE_HPP_