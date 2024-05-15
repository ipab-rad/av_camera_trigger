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
find_package(camera_trigger_msgs REQUIRED)

include_directories(include)

add_executable(camera_trigger_node 
    src/main.cpp
    src/camera_trigger_node.cpp
    src/device_interface.cpp
    src/hid_device_interface.cpp)

target_link_libraries(camera_trigger_node
    hidapi-libusb
    ${rclcpp_LIBRARIES}
)

ament_target_dependencies(camera_trigger_node rclcpp camera_trigger_msgs)

install(TARGETS
  camera_trigger_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
