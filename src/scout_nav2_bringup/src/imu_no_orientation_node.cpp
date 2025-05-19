#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuNoOrientationNode : public rclcpp::Node
{
public:
  ImuNoOrientationNode()
  : Node("imu_no_orientation_node")
  {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/out", 10,
      std::bind(&ImuNoOrientationNode::imuCallback, this, std::placeholders::_1));

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/no_orientation", 10);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto new_msg = *msg;
    new_msg.orientation.x = 0.0;
    new_msg.orientation.y = 0.0;
    new_msg.orientation.z = 0.0;
    new_msg.orientation.w = 0.0;

    new_msg.orientation_covariance[0] = -1.0;  // orientation 사용하지 않음 표시
    imu_pub_->publish(new_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNoOrientationNode>());
  rclcpp::shutdown();
  return 0;
}