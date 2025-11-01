#pragma once
#include "rclcpp/rclcpp.hpp"

namespace urg_node2_nl
{
class UrgNode2NL : public rclcpp::Node
{
public:
  explicit UrgNode2NL(const rclcpp::NodeOptions & options);
  ~UrgNode2NL();
};
}  // namespace urg_node2_nl
