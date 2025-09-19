#include "recovery_decision_node/topic_boolean_condition.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

TopicBooleanCondition::TopicBooleanCondition(
    const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), last_msg_data_(false)
{
    node_ = rclcpp::Node::make_shared("topic_boolean_condition_rclcpp_node");

    getInput("topic_name", topic_name_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
    topic_name_, qos,
    std::bind(&TopicBooleanCondition::topic_callback, this, std::placeholders::_1));

  // 백그라운드에서 이 노드가 ROS 메시지를 받을 수 있도록 스레드 생성
  std::thread([this]() { rclcpp::spin(node_); }).detach();
}

BT::PortsList TopicBooleanCondition::providedPorts()
{
    return { BT::InputPort<std::string>("topic_name", "The topic name to subscribe to") };
}

void TopicBooleanCondition::topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 메시지를 받으면 최신 값을 저장 (Thread-safe를 위해 mutex 사용)
  std::lock_guard<std::mutex> lock(mutex_);
  last_msg_data_ = msg->data;
}

BT::NodeStatus TopicBooleanCondition::tick()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_msg_data_ == true){
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<TopicBooleanCondition>("TopicBooleanCondition");
}