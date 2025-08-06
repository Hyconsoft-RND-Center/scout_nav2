#ifndef OBSTACLE_DETECTOR_NODE_HPP_
#define OBSTACLE_DETECTOR_NODE_HPP_

#include <chrono>
#include <functional>
#include <map>
#include <string>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  explicit ObstacleDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 콜백 함수들
  void sensor1_callback(const sensor_msgs::msg::Range::SharedPtr msg);
  void sensor2_callback(const sensor_msgs::msg::Range::SharedPtr msg);
  void sensor3_callback(const sensor_msgs::msg::Range::SharedPtr msg);
  void sensor4_callback(const sensor_msgs::msg::Range::SharedPtr msg);

  void check_and_publish_obstacles();
  void range_filter(const std::string &key, float range);

  // 멤버 변수들
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_sensor1_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_sensor2_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_sensor3_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_sensor4_;
  
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rear_obstacle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr side_obstacle_pub_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, float> sensor_ranges_;
  double detection_distance_;
  double max_sensor_range_;
  double min_sensor_range_;

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters);
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

#endif // OBSTACLE_DETECTOR_NODE_HPP_