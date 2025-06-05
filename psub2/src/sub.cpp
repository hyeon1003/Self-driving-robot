#include "psub2/sub.hpp"
void Sub::subscribe_msg(const std_msgs::msg::String::SharedPtr msg) const
{
RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
}
Sub::Sub() : Node("mysub")
{
auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
sub_ = this->create_subscription<std_msgs::msg::String>("mytopic", qos_profile,
std::bind(&Sub::subscribe_msg, this, _1));
}