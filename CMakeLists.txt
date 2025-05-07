cmake_minimum_required(VERSION 3.8)
project(dvrk_magewell)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ./include
)

# Add executable
add_executable(publish_video src/publish_video.cpp)
add_executable(display_video src/display_video.cpp src/utils.cpp)
add_executable(local_display_video src/local_display_video.cpp src/utils.cpp)

# Link libraries for publish_video
ament_target_dependencies(publish_video
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(publish_video
  ${OpenCV_LIBRARIES}
)

# Link libraries for display_video
ament_target_dependencies(display_video
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  image_transport
)

target_link_libraries(display_video
  ${OpenCV_LIBRARIES}
)

# Link libraries for local_display_video
ament_target_dependencies(local_display_video
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(local_display_video
 ${OpenCV_LIBRARIES})

# Install the node executables
install(TARGETS
  publish_video
  display_video
  local_display_video
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()