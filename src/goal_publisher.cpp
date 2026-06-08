#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct Goal {
  double x, y, theta;
};

class GoalPublisher : public rclcpp::Node
{
public:
  GoalPublisher()
  : Node("goal_publisher"), current_goal_idx_(0)
  {
    action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const auto csv_path_override = declare_parameter<std::string>("csv_path", "");

    client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    const std::string csv_path = csv_path_override.empty() ?
      ament_index_cpp::get_package_share_directory("second_project") + "/csv/goals.csv" :
      csv_path_override;

    load_goals(csv_path);

    if (goals_.empty()) {
      RCLCPP_ERROR(get_logger(), "No goals loaded from: %s", csv_path.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu goals from %s", goals_.size(), csv_path.c_str());
    RCLCPP_INFO(get_logger(), "Using action '%s' in frame '%s'",
      action_name_.c_str(), frame_id_.c_str());

    // Visualization: publish all goals once (latched) and the current goal as we go.
    goals_viz_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "goals", rclcpp::QoS(1).transient_local());
    current_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "current_goal", rclcpp::QoS(1).transient_local());
    publish_all_goals();

    // Poll until the action server is available, then start sending goals
    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() { try_connect(); });
  }

private:
  // ---------- CSV loading ----------
  void load_goals(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open CSV: %s", path.c_str());
      return;
    }

    std::string line;
    std::getline(file, line);  // skip header row

    while (std::getline(file, line)) {
      if (line.empty()) {continue;}
      std::istringstream ss(line);
      std::string token;
      Goal g{};
      try {
        std::getline(ss, token, ','); g.x     = std::stod(token);
        std::getline(ss, token, ','); g.y     = std::stod(token);
        std::getline(ss, token, ','); g.theta = std::stod(token);
        goals_.push_back(g);
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Skipping malformed CSV line '%s': %s",
          line.c_str(), e.what());
      }
    }
  }

  // ---------- Visualization helpers ----------
  geometry_msgs::msg::Pose make_pose(const Goal & g) const
  {
    geometry_msgs::msg::Pose p;
    p.position.x = g.x;
    p.position.y = g.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, g.theta);
    p.orientation = tf2::toMsg(q);
    return p;
  }

  void publish_all_goals()
  {
    geometry_msgs::msg::PoseArray arr;
    arr.header.frame_id = frame_id_;
    arr.header.stamp = now();
    for (const auto & g : goals_) {
      arr.poses.push_back(make_pose(g));
    }
    goals_viz_pub_->publish(arr);
  }

  // ---------- Action server connection ----------
  void try_connect()
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(get_logger(), "Waiting for navigate_to_pose action server...");
      return;
    }
    timer_->cancel();
    send_goal(current_goal_idx_);
  }

  // ---------- Goal sending ----------
  void send_goal(size_t idx)
  {
    if (idx >= goals_.size()) {
      RCLCPP_INFO(get_logger(), "All %zu goals completed!", goals_.size());
      return;
    }

    const auto & g = goals_[idx];
    RCLCPP_INFO(get_logger(), "Sending goal %zu/%zu: x=%.2f  y=%.2f  theta=%.3f",
      idx + 1, goals_.size(), g.x, g.y, g.theta);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = frame_id_;
    goal_msg.pose.header.stamp    = now();
    goal_msg.pose.pose.position.x = g.x;
    goal_msg.pose.pose.position.y = g.y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, g.theta);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    // Publish the active goal so RViz can show it.
    current_goal_pub_->publish(goal_msg.pose);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    opts.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & handle) {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Goal rejected by server – skipping");
          current_goal_idx_++;
          send_goal(current_goal_idx_);
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted");
        }
      };

    opts.feedback_callback =
      [this](GoalHandleNav::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_DEBUG(get_logger(), "Distance remaining: %.2f m",
          fb->distance_remaining);
      };

    opts.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal %zu SUCCEEDED", current_goal_idx_ + 1);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Goal %zu ABORTED – proceeding to next",
              current_goal_idx_ + 1);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Goal %zu CANCELED – proceeding to next",
              current_goal_idx_ + 1);
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Goal %zu unknown result code",
              current_goal_idx_ + 1);
            break;
        }
        current_goal_idx_++;
        send_goal(current_goal_idx_);
      };

    client_->async_send_goal(goal_msg, opts);
  }

  // ---------- Members ----------
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goals_viz_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<Goal> goals_;
  std::string action_name_;
  std::string frame_id_;
  size_t current_goal_idx_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPublisher>());
  rclcpp::shutdown();
  return 0;
}
