#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

int getch(void) {
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

bool kbhit(void) {
  struct termios oldt, newt;
  int ch, oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if (ch != EOF) {
    ungetc(ch, stdin);
    return true;
  }
  return false;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("node_pub_wsl");
  auto pub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", 10);
  geometry_msgs::msg::Vector3 vel;
  rclcpp::WallRate rate(20);

  int goal1 = 0, goal2 = 0, vel1 = 0, vel2 = 0;
  while (rclcpp::ok()) {
    if (kbhit()) {
      char c = getch();
      switch (c) {
        case 'f': goal1 = 50;  goal2 = -50; break;
        case 'b': goal1 = -50; goal2 = 50;  break;
        case 'l': goal1 = -50; goal2 = -50; break;
        case 'r': goal1 = 50;  goal2 = 50;  break;
        default:  goal1 = goal2 = 0;      break;
      }
    }

    vel1 += (goal1 > vel1) ? 5 : (goal1 < vel1) ? -5 : 0;
    vel2 += (goal2 > vel2) ? 5 : (goal2 < vel2) ? -5 : 0;

    vel.x = vel1;
    vel.y = vel2;
    RCLCPP_INFO(node->get_logger(), "Pub: %d, %d", vel1, vel2);
    pub->publish(vel);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
