#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>

using std::placeholders::_1;

struct Channel {
  std::string in_topic;
  std::string out_scan_topic;
  std::string out_cloud_topic; // 비우면 미사용
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;
};

class RangeFanBridge : public rclcpp::Node {
public:
  RangeFanBridge() : Node("range_fan_bridge")
  {
    // 파라미터 선언
    declare_parameter<std::vector<std::string>>("input_topics", {});
    declare_parameter<std::vector<std::string>>("output_scan_topics", {});
    declare_parameter<std::vector<std::string>>("output_cloud_topics", {});     // optional
    declare_parameter<int>("num_beams", 15);
    declare_parameter<double>("fov_deg", 30.0);
    declare_parameter<double>("exaggeration", 1.0);
    declare_parameter<double>("cloud_angle_step_deg", 3.0);
    declare_parameter<bool>("publish_cloud", false);
    declare_parameter<bool>("override_stamp", false);

    // 파라미터 로드
    input_topics_        = get_parameter("input_topics").as_string_array();
    output_scan_topics_  = get_parameter("output_scan_topics").as_string_array();
    output_cloud_topics_ = get_parameter("output_cloud_topics").as_string_array();
    num_beams_ = std::max<int>(1, static_cast<int>(get_parameter("num_beams").as_int()));    fov_deg_             = get_parameter("fov_deg").as_double();
    exaggeration_        = std::max(0.0, get_parameter("exaggeration").as_double());
    cloud_step_deg_      = std::max(0.1, get_parameter("cloud_angle_step_deg").as_double());
    publish_cloud_       = get_parameter("publish_cloud").as_bool();
    override_stamp_      = get_parameter("override_stamp").as_bool();

    // 토픽 배열 검증
    if (output_scan_topics_.size() != input_topics_.size()) {
      RCLCPP_FATAL(get_logger(), "input_topics와 output_scan_topics 길이는 같아야 합니다.");
      throw std::runtime_error("Topic array size mismatch");
    }
    if (!output_cloud_topics_.empty() &&
        output_cloud_topics_.size() != input_topics_.size()) {
      RCLCPP_FATAL(get_logger(), "output_cloud_topics 제공 시 input_topics 길이와 같아야 합니다.");
      throw std::runtime_error("Topic array size mismatch");
    }

    // QoS
    auto qos = rclcpp::SensorDataQoS().keep_last(10).best_effort().durability_volatile();

    // 채널 생성
    channels_.resize(input_topics_.size());
    for (size_t i = 0; i < input_topics_.size(); ++i) {
      channels_[i].in_topic        = input_topics_[i];
      channels_[i].out_scan_topic  = output_scan_topics_[i];
      channels_[i].out_cloud_topic = (!output_cloud_topics_.empty()) ? output_cloud_topics_[i] : std::string();

      channels_[i].pub_scan = create_publisher<sensor_msgs::msg::LaserScan>(
        channels_[i].out_scan_topic, qos);

      if (publish_cloud_ && !channels_[i].out_cloud_topic.empty()) {
        channels_[i].pub_cloud = create_publisher<sensor_msgs::msg::PointCloud2>(
          channels_[i].out_cloud_topic, qos);
      }

      channels_[i].sub = create_subscription<sensor_msgs::msg::Range>(
        channels_[i].in_topic, qos,
        [this, i](sensor_msgs::msg::Range::SharedPtr msg) {
          this->rangeCb(msg, i);
        });
    }

    RCLCPP_INFO(get_logger(),
      "range_fan_bridge ready: channels=%zu, beams=%d, fov=%.1f deg, ex=%.2f, cloud=%s",
      channels_.size(), num_beams_, fov_deg_, exaggeration_, publish_cloud_ ? "on":"off");
  }

private:
  void rangeCb(const sensor_msgs::msg::Range::SharedPtr msg, size_t idx)
  {
    if (idx >= channels_.size()) return;
    auto &ch = channels_[idx];

    // 원본 범위
    const double rmin = static_cast<double>(msg->min_range);
    const double rmax = static_cast<double>(msg->max_range);
    double r          = static_cast<double>(msg->range);

    // 유효성
    const bool invalid = (!std::isfinite(r) || r <= 0.0);
    if (!invalid) {
      r = std::clamp(r * exaggeration_, rmin, rmax);
    }

    // 각도 설정
    const double fov       = std::max(0.0, fov_deg_) * M_PI / 180.0;
    const double angle_min = -0.5 * fov;
    const double angle_max =  0.5 * fov;
    const double angle_inc = (num_beams_ > 1) ? (fov / static_cast<double>(num_beams_ - 1)) : fov;

    // LaserScan
    sensor_msgs::msg::LaserScan scan;
    scan.header = msg->header;
    if (override_stamp_) scan.header.stamp = now();
    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = angle_inc;
    scan.time_increment = 0.0f;
    scan.scan_time = 0.0f;               // 필요 시 0.01f(=100Hz)로 세팅 가능
    scan.range_min = rmin;
    scan.range_max = rmax;
    scan.ranges.resize(num_beams_);

    if (invalid) {
      std::fill(scan.ranges.begin(), scan.ranges.end(),
                std::numeric_limits<float>::infinity());
    } else {
      std::fill(scan.ranges.begin(), scan.ranges.end(), static_cast<float>(r));
    }
    ch.pub_scan->publish(scan);

    // PointCloud2 (옵션)
    if (publish_cloud_ && ch.pub_cloud) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header = msg->header;
      if (override_stamp_) cloud.header.stamp = now();
      cloud.height = 1;

      sensor_msgs::PointCloud2Modifier mod(cloud);
      mod.setPointCloud2FieldsByString(1, "xyz");

      if (invalid) {
        mod.resize(0);
        ch.pub_cloud->publish(cloud);
        return;
      }

      const double step = cloud_step_deg_ * M_PI / 180.0;
      const size_t npts = std::max<size_t>(1, static_cast<size_t>(std::floor(fov / step)) + 1);
      mod.resize(npts);

      sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");

      const double rr = std::clamp(r, rmin, rmax);
      for (size_t i = 0; i < npts; ++i, ++it_x, ++it_y, ++it_z) {
        const double a = angle_min + static_cast<double>(i) * step;
        *it_x = static_cast<float>(rr * std::cos(a));
        *it_y = static_cast<float>(rr * std::sin(a));
        *it_z = 0.0f;
      }
      ch.pub_cloud->publish(cloud);
    }
  }

private:
  // params
  std::vector<std::string> input_topics_;
  std::vector<std::string> output_scan_topics_;
  std::vector<std::string> output_cloud_topics_;
  int    num_beams_{15};
  double fov_deg_{30.0};
  double exaggeration_{1.0};
  double cloud_step_deg_{3.0};
  bool   publish_cloud_{false};
  bool   override_stamp_{false};

  // channels
  std::vector<Channel> channels_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangeFanBridge>());
  rclcpp::shutdown();
  return 0;
}
