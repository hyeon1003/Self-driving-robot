#include "line_tracer_wsl/wsl.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>

static bool compare_info(const Info &a, const Info &b) {
  return a.get_distance() < b.get_distance();
}

WslNode::WslNode()
: Node("wsl_node"),
  original_video_("original.mp4",
                  cv::VideoWriter::fourcc('m','p','4','v'),
                  30, cv::Size(640,360)),
  toUse_video_("toUse.mp4",
               cv::VideoWriter::fourcc('m','p','4','v'),
               30, cv::Size(640,90)),
  past_point_(320, 45),
  present_point_(320, 45)
{
  // 이미지 토픽 구독
  sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    "camera",
    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
    std::bind(&WslNode::img_show, this, _1)
  );

  // 에러 토픽 퍼블리시
  pub_ = create_publisher<std_msgs::msg::Int32>(
    "error",
    rclcpp::QoS(rclcpp::KeepLast(10))
  );

  // 33ms 주기로 pub_callback 호출
  timer_ = create_wall_timer(
    std::chrono::milliseconds(33),
    std::bind(&WslNode::pub_callback, this)
  );
}

void WslNode::img_show(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  // 이미지 디코딩
  original_ = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
  // 하단 1/4 영역만 처리
  toUse_ = original_(cv::Rect(0, original_.rows*3/4, original_.cols, original_.rows/4));

  // 그레이스케일 → 밝기 보정 → 이진화 → 레이블링
  cv::cvtColor(toUse_, toUse_, cv::COLOR_BGR2GRAY);
  toUse_ += 100 - cv::mean(toUse_)[0];
  cv::threshold(toUse_, toUse_, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  int label_cnt = cv::connectedComponentsWithStats(toUse_, labels_, stats_, centroids_);
  cv::cvtColor(toUse_, toUse_, cv::COLOR_GRAY2BGR);

  // 각 레이블 중심까지 거리 계산
  std::vector<Info> v;
  for (int i = 1; i < label_cnt; ++i) {
    double *c = centroids_.ptr<double>(i);
    int    *s = stats_.ptr<int>(i);
    if (s[4] > 100) {
      double dist = std::hypot(present_point_.x - c[0],
                               present_point_.y - c[1]);
      v.emplace_back(i, dist);
    }
  }

  // 가장 가까운 중심점 선택 및 점프 필터링
  if (!v.empty()) {
    std::sort(v.begin(), v.end(), compare_info);
    int idx = v.front().get_index();
    double *c = centroids_.ptr<double>(idx);
    present_point_ = cv::Point2d(c[0], c[1]);
    if (std::abs(present_point_.x - past_point_.x) > toUse_.cols/4 ||
        std::abs(present_point_.y - past_point_.y) > toUse_.rows/2)
    {
      present_point_ = past_point_;
    }
  }

  // 에러 계산 및 로깅
  error_.data = static_cast<int>(toUse_.cols/2 - present_point_.x);
  RCLCPP_INFO(get_logger(), "error: %d", error_.data);

  // 시각화: 나머지 레이블(파랑), 선택된 중심(빨강)
  for (size_t j = 1; j < v.size(); ++j) {
    int idx = v[j].get_index();
    double *c = centroids_.ptr<double>(idx);
    int    *s = stats_.ptr<int>(idx);
    cv::circle(toUse_, cv::Point(c[0], c[1]), 3, cv::Scalar(255,0,0), -1);
    cv::rectangle(toUse_, cv::Rect(s[0], s[1], s[2], s[3]), cv::Scalar(255,0,0));
  }
  cv::circle(toUse_, present_point_, 3, cv::Scalar(0,0,255), -1);

  // 화면 출력 및 녹화
  cv::imshow("original", original_);
  cv::imshow("toUse", toUse_);
  cv::waitKey(1);
  original_video_ << original_;
  toUse_video_     << toUse_;

  past_point_ = present_point_;
}

void WslNode::pub_callback()
{
  pub_->publish(error_);
}
