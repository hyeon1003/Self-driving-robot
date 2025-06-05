#ifndef _PUB_HPP_
#define _PUB_HPP_
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>
#include <chrono>
using namespace std::chrono_literals;
class Pub : public rclcpp::Node
{
    private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    size_t count_;
    int count;
    void publish_msg();
    public:
    Pub();
};
#endif //_PUB_HPP_