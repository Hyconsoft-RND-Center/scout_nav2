#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <fstream>
#include <filesystem>

class ClickedPointLogger : public rclcpp::Node
{
public:
    ClickedPointLogger()
    : Node("clicked_point_logger")
    {
        // 파라미터 선언 및 가져오기
        this->declare_parameter<std::string>("output_path", "/home/hycon/gazebo_nav/waypoints.csv");
        this->get_parameter("output_path", csv_path_);

        // 파일 초기화
        std::ofstream file(csv_path_, std::ios::out);
        file << "x,y,z\n";  // 헤더 작성
        file.close();

        // 구독 시작
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&ClickedPointLogger::point_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Logging clicked points to: %s", csv_path_.c_str());
    }

private:
    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        double x = msg->point.x;
        double y = msg->point.y;
        double z = msg->point.z;

        std::ofstream file(csv_path_, std::ios::app);  // append mode
        file << x << "," << y << "," << z << "\n";
        file.flush(); // 명시적으로 디스크에 기록
        file.close();

        RCLCPP_INFO(this->get_logger(), "Saved point: (%.2f, %.2f, %.2f)", x, y, z);
    }

    std::string csv_path_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClickedPointLogger>());
  rclcpp::shutdown();
  return 0;
}