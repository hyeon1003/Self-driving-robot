#include "psub2/sub.hpp"
int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
auto node = std::make_shared<Sub>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}