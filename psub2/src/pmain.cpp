#include "psub2/pub.hpp"
int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
auto node = std::make_shared<Pub>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}