#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("video_pub");
  auto pub  = node->create_publisher<sensor_msgs::msg::CompressedImage>("camera", 10);
  cv::VideoCapture cap("/home/jetson/simulation/7_lt_ccw_100rpm_in.mp4", cv::CAP_ANY); //in 코스 100rpm
  rclcpp::Rate rate(30);
  while (rclcpp::ok()) {
    cv::Mat frame; cap >> frame;
    if (frame.empty()) break;
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                 .toCompressedImageMsg();
    pub->publish(*msg);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
