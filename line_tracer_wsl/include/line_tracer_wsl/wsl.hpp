#ifndef LINE_TRACER_WSL__WSL_HPP_
#define LINE_TRACER_WSL__WSL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <chrono>
using std::placeholders::_1;

// 레이블 중심까지의 거리 정보를 담는 구조체
class Info {
private:
  int index_;
  double distance_;
public:
  Info(int idx, double dist)
  : index_(idx), distance_(dist) {}
  int get_index() const { return index_; }
  double get_distance() const { return distance_; }
};

// WSL2 노드: 이미지 구독 → 에러 계산 → 에러 퍼블리시
class WslNode : public rclcpp::Node {
public:
  WslNode();

private:
  void img_show(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void pub_callback();

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::Mat original_;
  cv::Mat toUse_;
  cv::VideoWriter original_video_;
  cv::VideoWriter toUse_video_;
  cv::Mat labels_, stats_, centroids_;
  cv::Point2d past_point_, present_point_;
  std_msgs::msg::Int32 error_;
};

#endif  // LINE_TRACER_WSL__WSL_HPP_
