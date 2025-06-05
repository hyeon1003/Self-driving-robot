#include "line_tracer_wsl/wsl.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WslNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
