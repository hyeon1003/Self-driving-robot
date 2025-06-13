#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "line_tracer_jetson/dxl.hpp"
using std::placeholders::_1;

class DxlCtrl : public rclcpp::Node {
public:
  DxlCtrl()
  : Node("dxl_ctrl"), dxl_()
  {
    if (!dxl_.open()) {
      RCLCPP_ERROR(get_logger(), "DXL open failed");
      rclcpp::shutdown();
      return;
    }
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "error", 10, std::bind(&DxlCtrl::on_error, this, _1));
  }

  ~DxlCtrl() {
    dxl_.close();
  }

private:
  void on_error(const std_msgs::msg::Int32::SharedPtr msg) {
    int err = msg->data;

    // 'line이 보이지 않을 때'를 의미하는 임계값: |err| > 300
    // if (std::abs(err) >270) {
    //   // 급회전: 왼쪽 45, 오른쪽 -145
    //   int lmotor = 50;
    //   int rmotor = -155;
    //   dxl_.setVelocity(lmotor, rmotor);
    //   RCLCPP_INFO(get_logger(),
    //               "Line lost (err=%d). Sharp turn → lmotor=%d, rmotor=%d",
    //               err, lmotor, rmotor);
    //   return;
    // }

    // 기본 속도 100을 기준으로, gain = 0.27
    double k = 0.27;
    int base_speed = 100;

    // 커브길에서 속도 차이를 주기 위한 공식:
    // 왼쪽 L =  100 - 0.27*err
    // 오른쪽 R = -100 - 0.27*err
    int lmotor = static_cast<int>(base_speed - k * err);
    int rmotor = static_cast<int>(-base_speed - k * err);

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
