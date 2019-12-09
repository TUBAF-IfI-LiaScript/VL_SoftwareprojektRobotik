#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_msg_package/msg/my_msg.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("heart_beat_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("heartBeat", 10);
  std_msgs::msg::String message;
  message.data = "Still alive" ;
  RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher->publish(message);
  rclcpp::shutdown();
  return 0;
}
