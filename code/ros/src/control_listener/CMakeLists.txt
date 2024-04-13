cmake_minimum_required(VERSION 3.8)
project(control_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(orca_ctrl_msgs REQUIRED)

add_executable(control_listener src/control_listener.cpp)
ament_target_dependencies(control_listener rclcpp control_msgs orca_ctrl_msgs)

add_executable(serial_comms src/serial_comms.cpp)
ament_target_dependencies(serial_comms rclcpp orca_ctrl_msgs)


install(TARGETS
  control_listener
  serial_comms
  DESTINATION lib/${PROJECT_NAME})



ament_package()
