#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "my_service/srv/robot_availability.hpp"

using RobotAvailability = my_service::srv::RobotAvailability;
rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  RCLCPP_INFO(
    g_node->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->robotname);
  response->location = "Kitchen";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("RobotAvailabilityService");
  auto server = g_node->create_service<RobotAvailability>("WhoIsWhere", handle_service);
  RCLCPP_INFO( g_node->get_logger(), "Robot Availability Service started ... ");
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
