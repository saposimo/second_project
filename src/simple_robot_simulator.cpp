#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}
}  // namespace

class SimpleRobotSimulator : public rclcpp::Node
{
public:
  SimpleRobotSimulator()
  : Node("simple_robot_simulator"),
    steady_clock_(RCL_STEADY_TIME)
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");

    x_ = declare_parameter<double>("initial_x", 1.6);
    y_ = declare_parameter<double>("initial_y", -4.4);
    yaw_ = declare_parameter<double>("initial_yaw", -0.013);
    robot_radius_ = declare_parameter<double>("robot_radius", 0.35);

    scan_range_min_ = declare_parameter<double>("scan_range_min", 0.1);
    scan_range_max_ = declare_parameter<double>("scan_range_max", 20.0);
    scan_angle_min_ = declare_parameter<double>("scan_angle_min", -kPi);
    scan_angle_max_ = declare_parameter<double>("scan_angle_max", kPi);
    scan_samples_ = declare_parameter<int>("scan_samples", 720);
    publish_frequency_ = declare_parameter<double>("publish_frequency", 20.0);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = msg;
        if (!map_ready_logged_) {
          RCLCPP_INFO(
            get_logger(), "Received map %u x %u at %.3f m/cell",
            map_->info.width, map_->info.height, map_->info.resolution);
          map_ready_logged_ = true;
        }
      });

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = steady_clock_.now();
      });

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(20));
    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_update_time_ = steady_clock_.now();
    last_cmd_time_ = last_update_time_;

    const auto period = std::chrono::duration<double>(1.0 / publish_frequency_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { update(); });

    RCLCPP_INFO(
      get_logger(),
      "Started internal 2D simulator: %s -> %s, %s, %s",
      cmd_vel_topic_.c_str(), odom_topic_.c_str(), scan_topic_.c_str(), map_topic_.c_str());
  }

private:
  builtin_interfaces::msg::Time current_stamp() const
  {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(sim_time_ns_ / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(sim_time_ns_ % 1000000000LL);
    return stamp;
  }

  void update()
  {
    const auto now = steady_clock_.now();
    const double dt = std::max(0.0, (now - last_update_time_).seconds());
    last_update_time_ = now;
    sim_time_ns_ += static_cast<int64_t>(dt * 1000000000.0);

    publish_clock();
    integrate_motion(dt, now);
    publish_tf_and_odom();

    if (map_) {
      publish_scan();
    } else if (!waiting_for_map_logged_) {
      RCLCPP_INFO(get_logger(), "Waiting for %s before publishing scans", map_topic_.c_str());
      waiting_for_map_logged_ = true;
    }
  }

  void publish_clock()
  {
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = current_stamp();
    clock_pub_->publish(clock_msg);
  }

  void integrate_motion(double dt, const rclcpp::Time & steady_now)
  {
    auto cmd = last_cmd_;
    if ((steady_now - last_cmd_time_).seconds() > cmd_vel_timeout_) {
      cmd = geometry_msgs::msg::Twist();
    }

    const double dx_world =
      (std::cos(yaw_) * cmd.linear.x - std::sin(yaw_) * cmd.linear.y) * dt;
    const double dy_world =
      (std::sin(yaw_) * cmd.linear.x + std::cos(yaw_) * cmd.linear.y) * dt;
    const double next_x = x_ + dx_world;
    const double next_y = y_ + dy_world;

    if (!map_ || !footprint_collides(next_x, next_y)) {
      x_ = next_x;
      y_ = next_y;
    }

    yaw_ = normalize_angle(yaw_ + cmd.angular.z * dt);
    current_vx_ = cmd.linear.x;
    current_vy_ = cmd.linear.y;
    current_wz_ = cmd.angular.z;
  }

  bool footprint_collides(double x, double y) const
  {
    if (!map_) {
      return false;
    }

    const double step = std::max<double>(map_->info.resolution, 0.05);
    for (double ox = -robot_radius_; ox <= robot_radius_; ox += step) {
      for (double oy = -robot_radius_; oy <= robot_radius_; oy += step) {
        if ((ox * ox + oy * oy) > robot_radius_ * robot_radius_) {
          continue;
        }
        if (is_occupied_world(x + ox, y + oy, true)) {
          return true;
        }
      }
    }
    return false;
  }

  bool world_to_map(double wx, double wy, int & mx, int & my) const
  {
    if (!map_) {
      return false;
    }

    const auto & info = map_->info;
    const double origin_x = info.origin.position.x;
    const double origin_y = info.origin.position.y;

    mx = static_cast<int>(std::floor((wx - origin_x) / info.resolution));
    my = static_cast<int>(std::floor((wy - origin_y) / info.resolution));

    return mx >= 0 && my >= 0 &&
      mx < static_cast<int>(info.width) &&
      my < static_cast<int>(info.height);
  }

  bool is_occupied_world(double wx, double wy, bool outside_is_occupied) const
  {
    int mx = 0;
    int my = 0;
    if (!world_to_map(wx, wy, mx, my)) {
      return outside_is_occupied;
    }

    const auto index = static_cast<size_t>(my) * map_->info.width + static_cast<size_t>(mx);
    const int8_t value = map_->data[index];
    return value >= 65;
  }

  double raycast(double angle) const
  {
    const double step = std::max<double>(map_->info.resolution, 0.05);
    for (double range = scan_range_min_; range <= scan_range_max_; range += step) {
      const double wx = x_ + range * std::cos(angle);
      const double wy = y_ + range * std::sin(angle);
      if (is_occupied_world(wx, wy, true)) {
        return range;
      }
    }
    return std::numeric_limits<float>::infinity();
  }

  void publish_scan()
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = current_stamp();
    scan.header.frame_id = base_frame_id_;
    scan.angle_min = scan_angle_min_;
    scan.angle_max = scan_angle_max_;
    scan.angle_increment = (scan_angle_max_ - scan_angle_min_) /
      static_cast<double>(scan_samples_ - 1);
    scan.time_increment = 0.0;
    scan.scan_time = 1.0 / publish_frequency_;
    scan.range_min = scan_range_min_;
    scan.range_max = scan_range_max_;
    scan.ranges.resize(static_cast<size_t>(scan_samples_));

    for (int i = 0; i < scan_samples_; ++i) {
      const double angle = yaw_ + scan_angle_min_ + i * scan.angle_increment;
      scan.ranges[static_cast<size_t>(i)] = static_cast<float>(raycast(angle));
    }

    scan_pub_->publish(scan);
  }

  void publish_tf_and_odom()
  {
    const auto stamp = current_stamp();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = transform.transform.rotation;
    odom.twist.twist.linear.x = current_vx_;
    odom.twist.twist.linear.y = current_vy_;
    odom.twist.twist.angular.z = current_wz_;
    odom_pub_->publish(odom);
  }

  rclcpp::Clock steady_clock_;
  rclcpp::Time last_update_time_;
  rclcpp::Time last_cmd_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  geometry_msgs::msg::Twist last_cmd_;

  std::string map_topic_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double robot_radius_{0.35};
  double scan_range_min_{0.1};
  double scan_range_max_{20.0};
  double scan_angle_min_{-kPi};
  double scan_angle_max_{kPi};
  double publish_frequency_{20.0};
  double cmd_vel_timeout_{0.5};
  double current_vx_{0.0};
  double current_vy_{0.0};
  double current_wz_{0.0};
  int scan_samples_{720};
  int64_t sim_time_ns_{0};
  bool waiting_for_map_logged_{false};
  bool map_ready_logged_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleRobotSimulator>());
  rclcpp::shutdown();
  return 0;
}
