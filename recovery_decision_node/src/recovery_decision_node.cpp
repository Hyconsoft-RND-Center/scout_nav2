#include "recovery_decision_node/recovery_decision_node.hpp"

RecoveryDecisionNode::RecoveryDecisionNode(const rclcpp::NodeOptions & options)
: Node("recovery_decision_node", options)
{
  obstacle_status_["frontal"] = false;
  obstacle_status_["left"] = false;
  obstacle_status_["right"] = false;
  obstacle_status_["rear"] = false;

  auto qos = rclcpp::SensorDataQoS();

  sub_frontal_ = this->create_subscription<std_msgs::msg::Bool>(
    "/is_frontal_obstacle", qos, std::bind(&RecoveryDecisionNode::frontal_obstacle_callback, this, std::placeholders::_1));
  sub_left_ = this->create_subscription<std_msgs::msg::Bool>(
    "/is_left_obstacle", qos, std::bind(&RecoveryDecisionNode::left_obstacle_callback, this, std::placeholders::_1));
  sub_right_ = this->create_subscription<std_msgs::msg::Bool>(
    "/is_right_obstacle", qos, std::bind(&RecoveryDecisionNode::right_obstacle_callback, this, std::placeholders::_1));
  sub_rear_ = this->create_subscription<std_msgs::msg::Bool>(
    "/is_rear_obstacle", qos, std::bind(&RecoveryDecisionNode::rear_obstacle_callback, this, std::placeholders::_1));

  service_ = this->create_service<recovery_interfaces::srv::GetRecoveryManeuver>(
    "/get_recovery_maneuver",
    std::bind(&RecoveryDecisionNode::get_recovery_maneuver_callback, this, std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(this->get_logger(), "Recovery Decision Node has been started.");
}

void RecoveryDecisionNode::frontal_obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg) { obstacle_status_["frontal"] = msg->data; }
void RecoveryDecisionNode::left_obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg) { obstacle_status_["left"] = msg->data; }
void RecoveryDecisionNode::right_obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg) { obstacle_status_["right"] = msg->data; }
void RecoveryDecisionNode::rear_obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg) { obstacle_status_["rear"] = msg->data; }

void RecoveryDecisionNode::get_recovery_maneuver_callback(
  const std::shared_ptr<recovery_interfaces::srv::GetRecoveryManeuver::Request> request,
  std::shared_ptr<recovery_interfaces::srv::GetRecoveryManeuver::Response>      response)
{
  (void)request;
  RCLCPP_INFO(this->get_logger(), "Received request for a recovery maneuver...");

  // 의사결정 로직
  if (obstacle_status_["frontal"]) {
    if (!obstacle_status_["left"] && !obstacle_status_["right"]) {
      RCLCPP_INFO(this->get_logger(), "Decision: Both sides clear. Suggesting SPIN LEFT.");
      response->command = "SPIN";
      response->spin_direction = 1.0f;
      response->spin_dist = 1.57f;
    } else if (!obstacle_status_["left"]) {
      RCLCPP_INFO(this->get_logger(), "Decision: Left clear. Suggesting SPIN LEFT.");
      response->command = "SPIN";
      response->spin_direction = 1.0f;
      response->spin_dist = 1.57f;
    } else if (!obstacle_status_["right"]) {
      RCLCPP_INFO(this->get_logger(), "Decision: Right clear. Suggesting SPIN RIGHT.");
      response->command = "SPIN";
      response->spin_direction = -1.0f;
      response->spin_dist = 1.57f;
    } else { // 앞, 좌, 우 모두 막힘
      if (!obstacle_status_["rear"]) {
        RCLCPP_INFO(this->get_logger(), "Decision: Trapped, but rear is clear. Suggesting BACKUP.");
        response->command = "BACKUP";
        response->backup_dist = 0.3f;
      } else {
        RCLCPP_WARN(this->get_logger(), "Decision: Completely trapped! No clear maneuver.");
        response->command = "WAIT"; // 할 수 있는 게 없음
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Recovery requested, but no frontal obstacle detected. Defaulting to WAIT.");
    response->command = "WAIT";
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<RecoveryDecisionNode>(options));
  rclcpp::shutdown();
  return 0;
}