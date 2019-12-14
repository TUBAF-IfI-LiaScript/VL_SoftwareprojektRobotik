cmake_minimum_required(VERSION 3.5)
project(my_services_tutorial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(my_services REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(service src/service.cpp)

ament_target_dependencies(
   service
   "rclcpp"
   "my_service")

install(
TARGETS service
DESTINATION lib/${PROJECT_NAME})

ament_package()
