#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <thread>

using namespace std::chrono_literals;

// --- 각도계산 함수 (정면이 0도, 오른쪽 +, 왼쪽 -) ---
double calc_angle(const cv::Point &pt, int cx, int cy) {
    int dx = pt.x - cx, dy = pt.y - cy;
    return std::atan2(dx, -dy) * 180.0 / M_PI;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // 1. 라이다 스캔 mp4 입력/출력 파일 지정
    std::string in_video = "sim1.mp4";
    std::string out_video = "sim_result.mp4";
    if (argc > 1) in_video = argv[1];
    if (argc > 2) out_video = argv[2];

    // 2. ROS2 노드 생성 및 퍼블리셔 생성 (/vel_cmd 토픽, std_msgs::msg::Int32)
    auto node = rclcpp::Node::make_shared("lidarsim");
    auto pub = node->create_publisher<std_msgs::msg::Int32>("/vel_cmd", 10);

    // 3. 입력 mp4 열기
    cv::VideoCapture cap(in_video);
    if (!cap.isOpened()) {
        std::cerr << "입력 영상 파일 열기 실패: " << in_video << std::endl;
        return -1;
    }
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0) fps = 10.0;
    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH), height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    if (width == 0 || height == 0) width = height = 500;

    // 4. 출력 mp4 저장 준비
    cv::VideoWriter writer(out_video, cv::VideoWriter::fourcc('m','p','4','v'), fps, cv::Size(width, height));
    if (!writer.isOpened()) {
        std::cerr << "출력 영상 파일 열기 실패: " << out_video << std::endl;
        return -1;
    }
    cv::namedWindow("LidarSim View", cv::WINDOW_AUTOSIZE);

    // 5. 프레임 반복 처리
    int cx = width/2, cy = height/2;
    while (rclcpp::ok()) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) break;
        cv::Mat img;
        if(frame.channels() == 3) {
            img = frame;
        } else {
            cv::cvtColor(frame, img, cv::COLOR_GRAY2BGR);
        }
        cv::Mat display = img.clone();

        // --- 장애물 좌/우 최단점 찾기 ---
        bool left_found=false, right_found=false;
        double min_left=1e9, min_right=1e9;
        cv::Point left_pt, right_pt;
        for (int y = 0; y < cy; ++y) {
            for (int x = 0; x < width; ++x) {
                cv::Vec3b p = img.at<cv::Vec3b>(y, x);
                if (p[2]>200 && p[1]<50 && p[0]<50) {
                    int dx = x-cx, dy = y-cy;
                    double d2 = dx*dx + dy*dy;
                    if (x <= cx && d2 < min_left) { min_left = d2; left_pt = cv::Point(x, y); left_found = true; }
                    if (x >= cx && d2 < min_right) { min_right = d2; right_pt = cv::Point(x, y); right_found = true; }
                }
            }
        }

        // --- error(각도, degree) 계산 ---
        double error = 0.0;
        if (left_found && right_found) {
            double la = calc_angle(left_pt, cx, cy);
            double ra = calc_angle(right_pt, cx, cy);
            error = (la + ra) / 2.0;
        } else if (left_found) {
            double la = calc_angle(left_pt, cx, cy);
            error = (la + 90.0) / 2.0;
        } else if (right_found) {
            double ra = calc_angle(right_pt, cx, cy);
            error = (-90.0 + ra) / 2.0;
        } // else 0도

        int error_int = static_cast<int>(std::round(error));

        // --- 퍼블리시 (/vel_cmd) ---
        std_msgs::msg::Int32 msg;
        msg.data = error_int;
        pub->publish(msg);

        // --- 시각화 ---
        // (1) 좌 장애물: 파란 점 & 파란 선
        if (left_found) {
            cv::circle(display, left_pt, 5, cv::Scalar(255,0,0), cv::FILLED);
            cv::line(display, cv::Point(cx,cy), left_pt, cv::Scalar(255,0,0), 2, cv::LINE_AA);
        }
        // (2) 우 장애물: 초록 점 & 초록 선
        if (right_found) {
            cv::circle(display, right_pt, 5, cv::Scalar(0,255,0), cv::FILLED);
            cv::line(display, cv::Point(cx,cy), right_pt, cv::Scalar(0,255,0), 2, cv::LINE_AA);
        }
        // (3) 중심점 crosshair
        cv::drawMarker(display, cv::Point(cx, cy), cv::Scalar(0,0,0), cv::MARKER_CROSS, 10, 1);

        // (4) 중앙 진행 방향(에러 각도): 굵은 검은 화살표
        double rad = error * CV_PI / 180.0;
        int len = 100;
        int dx = std::sin(rad) * len;
        int dy = std::cos(rad) * len;
        cv::arrowedLine(display, cv::Point(cx, cy), cv::Point(cx + dx, cy - dy), cv::Scalar(30,30,30), 4, cv::LINE_AA, 0, 0.3);

        cv::imshow("LidarSim View", display);
        cv::waitKey(1);
        writer.write(display);

        // 프레임 간 간격(10fps 기준)
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0/fps)));
    }
    cap.release();
    writer.release();
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
