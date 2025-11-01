#include "urg_node2_nl.hpp"

namespace urg_node2_nl
{
UrgNode2NL::UrgNode2NL(const rclcpp::NodeOptions & options) : Node("urg_node2_nl", options) {}
UrgNode2NL::~UrgNode2NL() {}
}  // namespace urg_node2_nl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg_node2_nl::UrgNode2NL)
