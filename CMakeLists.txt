cmake_minimum_required(VERSION 3.8)
project(lidar_target_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable for vision + LIDAR target follower
add_executable(vision_target_follower_node src/vision_target_follower_node.cpp)
ament_target_dependencies(vision_target_follower_node rclcpp sensor_msgs geometry_msgs cv_bridge)
target_link_libraries(vision_target_follower_node ${OpenCV_LIBS})

# Install executables
install(TARGETS
  vision_target_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install model files
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
