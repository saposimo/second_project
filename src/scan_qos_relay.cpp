#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanQosRelay : public rclcpp::Node
{
public:
  ScanQosRelay()
  : Node("scan_qos_relay")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan");

    const auto input_qos = rclcpp::SensorDataQoS();
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    output_qos.reliable();
    output_qos.durability_volatile();

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, output_qos);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_,
      input_qos,
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        publisher_->publish(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Relaying LaserScan from %s (Best Effort) to %s (Reliable)",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanQosRelay>());
  rclcpp::shutdown();
  return 0;
}
