#include <rclcpp/rclcpp.hpp>          
#include <std_msgs/msg/int32.hpp>     
#include <opencv2/opencv.hpp>         
#include <iostream>
#include <cmath>
#include <string>
#include <thread>

using namespace std::chrono_literals;



// pt가 중심을 기준으로 만드는 각도(degree, 0도=정면, +:오른쪽, -:왼쪽)
double calc_angle(const cv::Point &pt, int cx, int cy) {// 최소점과 중심점 간 각도 계산 함수
    int dx = pt.x - cx, dy = pt.y - cy;             // 중심에서의 상대 좌표
    return std::atan2(dx, -dy) * 180.0 / M_PI;      // atan2로 각도 계산, 위가 0도
}

// (cv::Point pt: 점 좌표, int cx, cy: 중심 좌표)

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    //라이다 mp4 입력/출력 파일명 지정 나중에 라이다로 sub 받을거임
    std::string in_video = "sim1.mp4";              
    std::string out_video = "sim_result.mp4";       

    // 노드, 퍼블리셔 생성 
    auto node = rclcpp::Node::make_shared("lidarsim");
    auto pub = node->create_publisher<std_msgs::msg::Int32>("/vel_cmd", 10);

    //입력 영상(mp4) 파일 열기
    cv::VideoCapture cap(in_video);
    if (!cap.isOpened()) {
        std::cerr << "입력 영상 파일 열기 실패: " << in_video << std::endl;
        return -1;
    }
    double fps = cap.get(cv::CAP_PROP_FPS);         // 영상 fps
    if (fps <= 0) fps = 10.0;                       // fps값 없으면 기본 10
    int width = cap.get(cv::CAP_PROP_FRAME_WIDTH), height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    if (width == 0 || height == 0) width = height = 500;   // 해상도 정보 없으면 500x500

    //결과 mp4 파일 저장 준비
    cv::VideoWriter writer(out_video, cv::VideoWriter::fourcc('m','p','4','v'), fps, cv::Size(width, height));
    if (!writer.isOpened()) {
        std::cerr << "출력 영상 파일 열기 실패: " << out_video << std::endl;
        return -1;
    }
    cv::namedWindow("LidarSim View", cv::WINDOW_AUTOSIZE); // 실시간 시각화 창

    int cx = width/2, cy = height/2;               // 라이다 중심 좌표 
    while (rclcpp::ok()) {                         
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) break;  // 다음 프레임 읽기, 끝나면 종료

        cv::Mat img;
        if(frame.channels() == 3) {
            img = frame;
        } else {
            cv::cvtColor(frame, img, cv::COLOR_GRAY2BGR);
        }
        
        cv::Mat display = img.clone();             // 시각화용 복사본

        //장애물(빨간 점) 좌/우 최단점 탐색
        bool left_found=false, right_found=false;   // 좌/우 장애물 유무
        double min_left=500, min_right=500;        // 중심점(라이다)와 가장 가까운 최소거리 초기화
        cv::Point left_pt, right_pt;               // 최단점 좌표

        // 중심에서 반경 MAX_RADIUS 이내만 인식
        const int MAX_RADIUS = 100;                // 50->100픽셀로 설정

        // 전방(상단) 픽셀을 모두 스캔하면서 빨간 점 탐색
        for (int y = 0; y < cy; ++y) {             // 프레임의 상단(전방)만 검사
            for (int x = 0; x < width; ++x) {
                cv::Vec3b p = img.at<cv::Vec3b>(y, x);
                // 빨간 점(장애물) 조건: R>200, G<50, B<50 이 조건없으면  빨간점을 못찾음 
                if (p[2]>200 && p[1]<50 && p[0]<50) {
                    int dx = x - cx, dy = y - cy;              // 중심점으로부터의 상대좌표
                    double d2 = dx*dx + dy*dy;                 
                    if (std::sqrt(d2) > MAX_RADIUS) continue;  // 반경 초과 시 무시하는 조건

                    // x좌표 기준: 좌/우 영역 분할, 가장 가까운 점만 저장
                    if (x <= cx && d2 < min_left) { min_left = d2; left_pt = cv::Point(x, y); left_found = true; } //왼쪽 
                    if (x >= cx && d2 < min_right) { min_right = d2; right_pt = cv::Point(x, y); right_found = true; }//오른쪽
                }
            }
        }

        // 에러계산
        double error = 0.0; //초기화
        if (left_found && right_found) { // 양쪽 장애물 모두 있을 때: 중앙(평균) 각도로 진행
            double la = calc_angle(left_pt, cx, cy);
            double ra = calc_angle(right_pt, cx, cy);
            error = (la + ra) / 2.0;
        } else if (left_found) {
            // 왼쪽만 장애물 있을 때: 좌측 각도+90의 평균
            double la = calc_angle(left_pt, cx, cy);
            error = (la + 90.0) / 2.0;
        } else if (right_found) {
            // 오른쪽만 장애물 있을 때: -90과 우측 각도의 평균
            double ra = calc_angle(right_pt, cx, cy);
            error = (-90.0 + ra) / 2.0;
        }
        // 둘 다 없으면 error = 0(직진)

        int error_int = int(error); // 정수형 변환

        // 퍼블리시 (/vel_cmd 토픽에 error 각도 값 전송)
        std_msgs::msg::Int32 msg;
        msg.data = error_int;
        pub->publish(msg);

        // 시각화
        // 좌측 장애물: 파란선 표시
        if (left_found) {
            cv::circle(display, left_pt, 5, cv::Scalar(255,0,0), cv::FILLED);
            cv::line(display, cv::Point(cx,cy), left_pt, cv::Scalar(255,0,0), 2, cv::LINE_AA);
        }
        // 우측 장애물: 초록선표시
        if (right_found) {
            cv::circle(display, right_pt, 5, cv::Scalar(0,255,0), cv::FILLED);
            cv::line(display, cv::Point(cx,cy), right_pt, cv::Scalar(0,255,0), 2, cv::LINE_AA);
        }
        // 중심점 표시
        cv::drawMarker(display, cv::Point(cx, cy), cv::Scalar(0,0,0), cv::MARKER_CROSS, 10, 1);

        // 중앙 진행 방향 화살표 표시
        double rad = error * CV_PI / 180.0;           // 각도를 라디안으로 변환
        int len = 100;                                // 화살표 길이(픽셀)
        int dx = std::sin(rad) * len;                 // x방향 벡터
        int dy = std::cos(rad) * len;                 // y방향 벡터
        cv::arrowedLine(display, cv::Point(cx, cy), cv::Point(cx + dx, cy - dy), cv::Scalar(30,30,30), 4, cv::LINE_AA, 0, 0.1);

        // 영상 출력 및 mp4로 저장
        cv::imshow("LidarSim View", display);         // 실시간 표시
        cv::waitKey(1);                               // 1ms 대기(키입력 무시)
        writer.write(display);                        // mp4로 저장
        RCLCPP_INFO(node->get_logger(), "[Error(중앙 진행각)]: %d", int(error));
        if (left_found) {
            RCLCPP_INFO(node->get_logger(), "[좌측 최단점 좌표]: (%d, %d)", left_pt.x, left_pt.y);
        }
        if (right_found) {
            RCLCPP_INFO(node->get_logger(), "[우측 최단점 좌표]: (%d, %d)", right_pt.x, right_pt.y);
        }
        // ROS2 이벤트 처리 및 fps
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0/fps)));
    }
    // 후처리
    cap.release();
    writer.release();
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
