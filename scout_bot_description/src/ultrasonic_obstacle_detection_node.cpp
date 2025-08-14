#include "ultrasonic_obstacle_detection_cpp/ultrasonic_obstacle_detection_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <cmath>
#include <sstream>
#include <iomanip>

// 생성자: 노드의 모든 초기화 작업을 수행
ObstacleDetectorNode::ObstacleDetectorNode(const rclcpp::NodeOptions & options)
: Node("obstacle_detector_node", options)
{
  // 파라미터 선언 및 초기화
  this->declare_parameter<double>("detection_distance", 0.3);
  this->declare_parameter<double>("max_sensor_range", 5.0);
  this->declare_parameter<double>("min_sensor_range", 0.02);
  this->declare_parameter<std::string>("left_sensor_topic", "/ultrasonic_sensor_2");
  this->declare_parameter<std::string>("right_sensor_topic", "/ultrasonic_sensor_1");
  this->declare_parameter<std::string>("rear_left_sensor_topic", "/ultrasonic_sensor_3");
  this->declare_parameter<std::string>("rear_right_sensor_topic", "/ultrasonic_sensor_4");  

  this->get_parameter("detection_distance", detection_distance_);
  this->get_parameter("max_sensor_range", max_sensor_range_);
  this->get_parameter("min_sensor_range", min_sensor_range_);

  RCLCPP_INFO(this->get_logger(), "detection_distance: %.2f m", detection_distance_);
  RCLCPP_INFO(this->get_logger(), "max_sensor_range: %.2f m", max_sensor_range_);
  RCLCPP_INFO(this->get_logger(), "min_sensor_range: %.2f m", min_sensor_range_);

  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleDetectorNode::parameters_callback, this, std::placeholders::_1));

  // 센서 거리 값 맵 초기화
  sensor_ranges_["left"] = std::numeric_limits<float>::infinity();
  sensor_ranges_["right"] = std::numeric_limits<float>::infinity();
  sensor_ranges_["rear_left"] = std::numeric_limits<float>::infinity();
  sensor_ranges_["rear_right"] = std::numeric_limits<float>::infinity();
  
  // QoS
  auto sensor_qos = rclcpp::SensorDataQoS();
  
  std::string left_topic = this->get_parameter("left_sensor_topic").as_string();
  std::string right_topic = this->get_parameter("right_sensor_topic").as_string();
  std::string rear_left_topic = this->get_parameter("rear_left_sensor_topic").as_string();
  std::string rear_right_topic = this->get_parameter("rear_right_sensor_topic").as_string();

  // 구독자(Subscriber) 생성
  sub_sensor1_ = this->create_subscription<sensor_msgs::msg::Range>(
    left_topic, sensor_qos, std::bind(&ObstacleDetectorNode::sensor1_callback, this, std::placeholders::_1));
  sub_sensor2_ = this->create_subscription<sensor_msgs::msg::Range>(
    right_topic, sensor_qos, std::bind(&ObstacleDetectorNode::sensor2_callback, this, std::placeholders::_1));
  sub_sensor3_ = this->create_subscription<sensor_msgs::msg::Range>(
    rear_left_topic, sensor_qos, std::bind(&ObstacleDetectorNode::sensor3_callback, this, std::placeholders::_1));
  sub_sensor4_ = this->create_subscription<sensor_msgs::msg::Range>(
    rear_right_topic, sensor_qos, std::bind(&ObstacleDetectorNode::sensor4_callback, this, std::placeholders::_1));

  // 발행자(Publisher) 생성
  rear_obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("/is_rear_obstacle", 10);
  side_obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>("/is_side_obstacle", 10);

  // 타이머 생성 (100ms, 즉 10Hz 주기로 콜백 함수 실행)
  timer_ = this->create_wall_timer(
    100ms, std::bind(&ObstacleDetectorNode::check_and_publish_obstacles, this));
}

void ObstacleDetectorNode::range_filter(const std::string& key, float range)
{
  if (range >= min_sensor_range_ && range <= max_sensor_range_) {
    sensor_ranges_[key] = range;
  } else {
    sensor_ranges_[key] = (float)max_sensor_range_;
  }
}

// 각 센서 콜백 함수: 메시지를 받으면 맵에 거리 값을 저장
void ObstacleDetectorNode::sensor1_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  range_filter("left", msg-> range);
}

void ObstacleDetectorNode::sensor2_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  range_filter("right", msg-> range);
}

void ObstacleDetectorNode::sensor3_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  range_filter("rear_left", msg-> range);
}

void ObstacleDetectorNode::sensor4_callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  range_filter("rear_right", msg-> range);
}


rcl_interfaces::msg::SetParametersResult ObstacleDetectorNode::parameters_callback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true; // 일단 성공으로 초기화

  for (const auto &parameter : parameters) {
    std::string param_name = parameter.get_name();
    if (param_name == "detection_distance") {
      detection_distance_ = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Parameter 'detection_distance' set to: %.2f", detection_distance_);
    } else if (param_name == "max_sensor_range") {
      max_sensor_range_ = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Parameter 'max_sensor_range' set to: %.2f", max_sensor_range_);
    } else if (param_name == "min_sensor_range") {
      min_sensor_range_ = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "Parameter 'min_sensor_range' set to: %.2f", min_sensor_range_);
    }
  }
  return result;
}

// 타이머 콜백 함수: 주기적으로 장애물 상태를 점검하고 결과를 발행
void ObstacleDetectorNode::check_and_publish_obstacles()
{
  // 파라미터 값이 동적으로 변경될 수 있으므로 주기적으로 값을 가져옴
  bool rear_obstacle_detected = (sensor_ranges_["rear_left"] < detection_distance_ ||
                                 sensor_ranges_["rear_right"] < detection_distance_);
  
  bool side_obstacle_detected = (sensor_ranges_["left"] < detection_distance_ ||
                                 sensor_ranges_["right"] < detection_distance_);

  auto rear_msg = std_msgs::msg::Bool();
  rear_msg.data = rear_obstacle_detected;
  rear_obstacle_pub_->publish(rear_msg);

  auto side_msg = std_msgs::msg::Bool();
  side_msg.data = side_obstacle_detected;
  side_obstacle_pub_->publish(side_msg);

  if (rear_obstacle_detected || side_obstacle_detected)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), 
      "Obstacle Detected! Rear: " << std::boolalpha << rear_obstacle_detected 
      << ", Side: " << std::boolalpha << side_obstacle_detected
      << " (L: " << std::fixed << std::setprecision(2) << sensor_ranges_["left"]
      << ", R: " << sensor_ranges_["right"]
      << ", RL: " << sensor_ranges_["rear_left"]
      << ", RR: " << sensor_ranges_["rear_right"] << ")");
  }
}

// main 함수: ROS2를 초기화하고 노드를 실행(spin)
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>(options));
  rclcpp::shutdown();
  return 0;
}