#ifndef _JETSON_HPP_
#define _JETSON_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
//#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/int32.hpp"
#include "line_tracer_jetson/dxl.hpp"
#include <memory>
#include <chrono>
#include <functional>
#include <thread>
 
using namespace std::chrono_literals;
using std::placeholders::_1;
 
class jetson : public rclcpp::Node 
{
private:
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer_; //타이머 이벤트 주기와 콜백함수가 등록된 shared_ptr 객체
    cv::VideoCapture cap;
    cv::Mat img;
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
 
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub;
    std_msgs::msg::Int32 error;
    bool tf = false;
 
    Dxl dxl;
 
    void img_pub();
    void sub_callback(const std_msgs::msg::Int32::SharedPtr msg);
 
public:
    jetson();
    ~jetson();
};
#endif //_JETSON_HPP_
 