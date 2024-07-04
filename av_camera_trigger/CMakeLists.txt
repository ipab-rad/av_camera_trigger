cmake_minimum_required(VERSION 3.5)
project(av_camera_trigger)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(camera_trigger_msgs REQUIRED)

include_directories(include)

add_library(
  camera_trigger_node_component SHARED
  src/camera_trigger_node.cpp src/device_interface.cpp
  src/hid_device_interface.cpp)

target_link_libraries(camera_trigger_node_component hidapi-libusb
                      ${rclcpp_LIBRARIES})

rclcpp_components_register_node(
  camera_trigger_node_component PLUGIN "av_camera_trigger::CameraTriggerNode"
  EXECUTABLE camera_trigger)

ament_target_dependencies(camera_trigger_node_component camera_trigger_msgs
                          rclcpp rclcpp_components)

ament_export_targets(export_camera_trigger_node_component)
install(
  TARGETS camera_trigger_node_component
  EXPORT export_camera_trigger_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
