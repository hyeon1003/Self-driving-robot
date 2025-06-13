#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

// GStreamer 파이프라인: 네트워크 스트리밍용
std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! "
                  "nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! "
                  "h264parse ! rtph264pay pt=96 ! "
                  "udpsink host=203.234.58.166 port=8001 sync=false";

// 두 개의 VideoWriter 선언
cv::VideoWriter streamer_writer;  // GStreamer용
cv::VideoWriter file_writer;      // 파일 저장용

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 1. 디코딩
    cv::Mat color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (color.empty()) {
        RCLCPP_WARN(node->get_logger(), "Failed to decode image");
        return;
    }


    cv::Mat gray, binary, binary_bgr;
    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY);
    cv::cvtColor(binary, binary_bgr, cv::COLOR_GRAY2BGR);

    streamer_writer << binary_bgr; 
    file_writer << binary_bgr;

    RCLCPP_INFO(node->get_logger(), "Streamed & Saved Frame: %dx%d", binary.cols, binary.rows);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("sub_jetson1_2");

    
    streamer_writer.open(dst, 0, 30, cv::Size(640, 360), true);
    if (!streamer_writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "GStreamer writer open failed!");
        rclcpp::shutdown();
        return -1;
    }

  
    file_writer.open("output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(640, 360), true);
    if (!file_writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "File writer open failed!");
        rclcpp::shutdown();
        return -1;
    }

    
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile,fn);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
