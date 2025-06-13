#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host = 203.234.58.166 port = 8001 sync=false";

cv::VideoWriter writer;
        

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv::Mat gray, binary, binary_bgr;

    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY);
    cv::cvtColor(binary, binary_bgr, cv::COLOR_GRAY2BGR); 
    writer << binary_bgr;  
    RCLCPP_INFO(node->get_logger(), "Processed Binary Image: jpg,%d,%d", binary.cols, binary.rows);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub");
    writer.open(dst, 0, (double)30, cv::Size(640, 360), true);
    if(!writer.isOpened()) { RCLCPP_ERROR(node->get_logger(), "Writer open failed!"); rclcpp::shutdown(); return -1; }
    //auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed",qos_profile,fn);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

