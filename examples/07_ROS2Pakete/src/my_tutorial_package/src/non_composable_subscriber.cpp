/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

 #include <memory>

 #include "rclcpp/rclcpp.hpp"
 #include "std_msgs/msg/string.hpp"
 #include "my_msg_package/msg/my_msg.hpp"

rclcpp::Node::SharedPtr g_node = nullptr;

void topic_callback(const my_msg_package::msg::MyMsg::SharedPtr msg)
{
  RCLCPP_INFO(g_node->get_logger(), "I heard: %s - %d", msg->comment.c_str(), msg->counter);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  g_node = rclcpp::Node::make_shared("minimal_subscriber");
  auto subscription =
    g_node->create_subscription<my_msg_package::msg::MyMsg>("topic", 10, topic_callback);
  RCLCPP_INFO(g_node->get_logger(), "Ok, let's start the MyMsg.msg listener!");
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  subscription = nullptr;
  g_node = nullptr;
  return 0;
}
