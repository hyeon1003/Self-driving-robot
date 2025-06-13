#linetrace_sub.cpp


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "line_tracer_jetson/dxl.hpp"
using std::placeholders::_1;

class DxlCtrl : public rclcpp::Node {
public:
    DxlCtrl()
        : Node("vel_cmd"), dxl_()
    {
        if (!dxl_.open()) {
            RCLCPP_ERROR(get_logger(), "DXL open failed");
            rclcpp::shutdown();
            return;
        }
        sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/vel_cmd", 10, std::bind(&DxlCtrl::on_error, this, _1));
    }

    ~DxlCtrl() {
        dxl_.close();
    }

private:
    void on_error(const std_msgs::msg::Int32::SharedPtr msg) {
        int err = msg->data;

        // 기본 속도 100을 기준으로, gain = 0.25
        double k = 0.25;
        int base_speed = 100;

        // 커브길에서 속도 차이를 주기 위한 공식
        int lmotor = static_cast<int>(base_speed - k * err);
        int rmotor = static_cast<int>(-base_speed - k * err);

        // 역주행을 방지하기 위해 왼쪽이 음수가 되면 0으로 고정
        if (lmotor < 0) {
            lmotor = 0;
        }
        // 오른쪽이 양수가 되면 0으로 고정 (오른쪽은 음수일수록 앞으로)
        if (rmotor > 0) {
            rmotor = 0;
        }

        // 둘 다 0이 되면 아주 짧게 전진하게끔 최소값을 넣어줌
        if (lmotor == 0 && rmotor == 0) {
            // 최소 속도를 약간 주면 정지 상태를 벗어나 계속 전진
            lmotor = 10;
            rmotor = -10;
        }

        dxl_.setVelocity(lmotor, rmotor);
        RCLCPP_INFO(get_logger(),
            "err=%4d   lmotor=%4d   rmotor=%4d",
            err, lmotor, rmotor);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    Dxl dxl_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DxlCtrl>());
    rclcpp::shutdown();
    return 0;
}

