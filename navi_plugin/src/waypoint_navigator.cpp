#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <fstream>
#include <sstream>

class WaypointNavigator : public rclcpp::Node
{
public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    WaypointNavigator()
    : Node("waypoint_navigator")
    {
        this->declare_parameter<std::string>("csv_path", "/home/hycon/gazebo_nav/waypoints.csv");
        this->get_parameter("csv_path", csv_path_);
        use_sim_time_ = this->get_parameter_or("use_sim_time", false);
        
        client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "/navigate_through_poses");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&WaypointNavigator::send_goal, this)
        );
    }

private:
    std::vector<geometry_msgs::msg::PoseStamped> load_waypoints(const std::string& file_path)
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s", file_path.c_str());
            return {};
        }
        std::string line;
        std::getline(file, line);  // skip header

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string x_str, y_str, z_str;
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            std::getline(ss, z_str, ',');

            double x = std::stod(x_str);
            double y = std::stod(y_str);
            double z = std::stod(z_str);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation.w = 1.0;
            poses.push_back(pose);
        }

        return poses;
    }

    void send_goal()
    {
        timer_->cancel();

        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto poses = load_waypoints(csv_path_);
        if (poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "No poses loaded from file: %s", csv_path_.c_str());
            return;
        }

        auto goal_msg = NavigateThroughPoses::Goal();
        goal_msg.poses = poses;

        auto options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        options.result_callback = [this](const GoalHandle::WrappedResult & result) {
            RCLCPP_INFO(this->get_logger(), "Navigation completed with status: %d", result.code);
        };

        client_->async_send_goal(goal_msg, options);
    }

    std::string csv_path_;
    bool use_sim_time_;
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


// TODO: make this node doesn't work once