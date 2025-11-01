#include "urg_node2_nl.hpp"

namespace urg_node2_nl
{
UrgNode2NL::UrgNode2NL(const rclcpp::NodeOptions & options) : Node("urg_node2_nl", options)
{
  std::signal(SIGPIPE, SIG_IGN);

  // declare parameters
  declare_urgnode2_parameters();

  if (config_.use_multiecho_) {
    // TODO: implement multiecho func
  } else {
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(20));
  }

  start_thread();
}
UrgNode2NL::~UrgNode2NL() {}

void UrgNode2NL::declare_urgnode2_parameters()
{
  config_.ip_address_ = declare_parameter<std::string>("ip_address", "");
  config_.ip_port_ = declare_parameter<int>("ip_port", 10940);
  config_.serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  config_.serial_baud_ = declare_parameter<int>("serial_baud", 115200);
  config_.frame_id_ = declare_parameter<std::string>("frame_id", "laser");
  config_.calibrate_time_ = declare_parameter<bool>("calibrate_time", false);
  config_.synchronize_time_ = declare_parameter<bool>("synchronize_time", false);
  config_.publish_intensity_ = declare_parameter<bool>("publish_intensity", false);
  config_.publish_multiecho_ = declare_parameter<bool>("publish_multiecho", false);
  config_.error_limit_ = declare_parameter<int>("error_limit", 4);
  config_.error_reset_period_ = declare_parameter<double>("error_reset_period", 5.0),
  config_.diagnostics_tolerance_ = declare_parameter<double>("diagnostics_tolerance", 0.05);
  config_.diagnostics_window_time_ = declare_parameter<double>("diagnostics_window_time", 5.0);
  config_.time_offset_ = declare_parameter<double>("time_offset", 0.0);
  config_.angle_min_ = declare_parameter<double>("angle_min", -M_PI);
  config_.angle_max_ = declare_parameter<double>("angle_max", M_PI);
  config_.skip_ = declare_parameter<int>("skip", 0);
  config_.cluster_ = declare_parameter<int>("cluster", 1);
}

void UrgNode2NL::scan_thread_func()
{
  while (close_thread_flag_.load()) {
  }
}

void UrgNode2NL::start_thread()
{
  // clear flag for thread finish
  close_thread_flag_.store(false);
  scan_thread_ = std::thread(std::bind(&UrgNode2NL::scan_thread_func, this));
}

void UrgNode2NL::close_thread()
{
  // set thread finish
  close_thread_flag_.store(true);

  // join thread
  if (scan_thread_.joinable()) {
    scan_thread_.join();
  }
}

}  // namespace urg_node2_nl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg_node2_nl::UrgNode2NL)
