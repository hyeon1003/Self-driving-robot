#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

cv::VideoWriter writer;

void mysub_callback(
    rclcpp::Node::SharedPtr node, 
    const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) return;

    
    if (writer.isOpened()) {
        writer.write(frame);
    }

    // 시각화
    cv::Mat gray, binary;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);

    cv::imshow("Original", frame);
    cv::imshow("Gray", gray);
    cv::imshow("Binary", binary);
    cv::waitKey(1);

    RCLCPP_INFO(node->get_logger(), "Received Image : %s, %d, %d", msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

   
    int width = 640, height = 480;
    double fps = 30.0; 
    std::string output_file = "output2-2.mp4";
    int fourcc = cv::VideoWriter::fourcc('m','p','4','v'); 

    writer.open(output_file, fourcc, fps, cv::Size(width, height), true);
    if (!writer.isOpened()) {
        std::cerr << "Failed to open VideoWriter!" << std::endl;
        return -1;
    }

    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn
    );

    rclcpp::spin(node);
    writer.release();
    cv::destroyAllWindows();

    rclcpp::shutdown();
    return 0;
}
