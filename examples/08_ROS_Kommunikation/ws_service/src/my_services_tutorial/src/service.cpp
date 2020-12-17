#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "my_services/srv/robot_availability.hpp"

using RobotAvailability = my_services::srv::RobotAvailability;
rclcpp::Node::SharedPtr g_node = nullptr;

struct Robot{
    public:
        int roomNumber;
        std::string name;
        float batterylevel;
        bool busy;
    public:
        Robot(int number, std::string robot_name, float battery, bool actualbusy){
            roomNumber=number; name=robot_name; batterylevel=battery; busy=actualbusy;}
};

std::string locations[4] = { "Kitchen", "Livingroom", "Bedroom", "Bathroom"};

std::vector<Robot> RobotDB {
    Robot(0, "Erika", 11.232, true),
    Robot(1, "Maja", 12.0, false),
    Robot(2, "Julius", 4.0, true),
    Robot(3, "Alexander", 9.8, false),
};

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RobotAvailability::Request> request,
  const std::shared_ptr<RobotAvailability::Response> response)
{
  bool gotIt = false;
  (void)request_header;  // um warning zu vermeiden
  RCLCPP_INFO(
    g_node->get_logger(),
    "Received request for : %s", request->robotname.c_str());
  for (auto const& robot: RobotDB){
    if (request->robotname == robot.name){
      response->validname = true;
      response->location = locations[robot.roomNumber];
      response->busy = robot.busy;
      response->batterylevel = robot.batterylevel;
      gotIt = true;
      break;
    }
  }
  if (gotIt == false){
    RCLCPP_INFO(
      g_node->get_logger(),
      "No entry for %s in robot data base", request->robotname.c_str());
  }
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
