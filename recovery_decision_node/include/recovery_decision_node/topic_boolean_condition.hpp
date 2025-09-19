#ifndef TOPIC_BOOLEAN_CONDITION_HPP_
#define TOPIC_BOOLEAN_CONDITION_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "std_msgs/msg/bool.hpp"

class TopicBooleanCondition : public BT::ConditionNode
{
public:
  TopicBooleanCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  // BT 노드가 실행될 때 호출되는 핵심 함수
  BT::NodeStatus tick() override;

  // 이 노드가 사용하는 포트(입력)를 정의
  static BT::PortsList providedPorts();

private:
  void topic_callback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  std::string topic_name_;
  bool last_msg_data_;
  std::mutex mutex_;
};

#endif // TOPIC_BOOLEAN_CONDITION_HPP_