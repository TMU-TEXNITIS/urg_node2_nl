#pragma once
#include <csignal>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace urg_node2_nl
{
struct UrgNode2Config
{
  // ---- ros2 parameters ----
  std::string ip_address_;
  int ip_port_;
  std::string serial_port_;
  int serial_baud_;
  std::string frame_id_;
  bool calibrate_time_;
  bool synchronize_time_;
  bool publish_intensity_;
  bool publish_multiecho_;
  int error_limit_;
  double error_reset_period_;
  double diagnostics_tolerance_;
  double diagnostics_window_time_;
  double time_offset_;
  double angle_min_;
  double angle_max_;
  int skip_;
  int cluster_;

  // ---- configurations without ros2 parameter ----
  bool use_multiecho_{false};
};

class UrgNode2NL : public rclcpp::Node
{
public:
  explicit UrgNode2NL(const rclcpp::NodeOptions & options);
  ~UrgNode2NL();

private:
  void declare_urgnode2_parameters();

  void scan_thread_func();
  void start_thread();
  void close_thread();

  UrgNode2Config config_;

  std::thread scan_thread_;
  std::atomic_bool close_thread_flag_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> scan_pub_;
};
}  // namespace urg_node2_nl
