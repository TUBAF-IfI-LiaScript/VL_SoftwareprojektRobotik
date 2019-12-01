cmake_minimum_required(VERSION 3.5)
project(my_msg_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# Abhängigkeiten für Message-Generierung
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 
rosidl_generate_interfaces(new_msg
  "msg/MyMsg.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
