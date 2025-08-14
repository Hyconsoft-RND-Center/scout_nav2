#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <fstream>
#include <sstream>
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class WaypointMarkerPublisher : public rclcpp::Node
{
public:
    WaypointMarkerPublisher() : Node("waypoint_marker_publisher")
    {
        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/waypoints", 10,
            std::bind(&WaypointMarkerPublisher::waypointsCallback, this, std::placeholders::_1)
        );
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/waypoints_marker", 10
        );
    }

private:
    void waypointsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header = msg->header;
            marker.ns = "waypoint";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = msg->poses[i];
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;
            marker.color.r = 0.2;
            marker.color.g = 1.0;
            marker.color.b = 0.4;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header = msg->header;
            text_marker.ns = "waypoint_text";
            text_marker.id = 1000 + i;  // id는 Sphere와 안 겹치게 (예: 1000+i)
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose = msg->poses[i];
            text_marker.pose.position.z += 0.2;  // 점 위에 뜨게 약간 올림
            text_marker.scale.z = 0.13;          // 글씨 크기 (z만 사용)
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = std::to_string(i); // 표시할 텍스트 (인덱스)
            marker_array.markers.push_back(text_marker); 
        }
        if (msg->poses.size() > 1) {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header = msg->header;
            line_marker.ns = "waypoint_line";
            line_marker.id = 2000; // id는 한 개만 쓰면 됨
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.pose.orientation.w = 1.0; // 회전 없음
            line_marker.scale.x = 0.03; // 선 두께
            line_marker.color.r = 0.0;
            line_marker.color.g = 0.0;
            line_marker.color.b = 1.0;
            line_marker.color.a = 1.0;
            for (const auto& pose : msg->poses)
                line_marker.points.push_back(pose.position); // 모든 점을 차례대로 추가
            marker_array.markers.push_back(line_marker);
        }
        marker_pub_->publish(marker_array);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointMarkerPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
