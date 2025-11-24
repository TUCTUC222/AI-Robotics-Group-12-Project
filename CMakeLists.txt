cmake_minimum_required(VERSION 3.5)
project(my_world)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

########## Build plugin ##########
add_library(cylinder_auto_mover SHARED src/cylinder_auto_mover.cpp)

target_include_directories(cylinder_auto_mover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cylinder_auto_mover gazebo_dev)

########## Install plugin ##########
install(
  TARGETS cylinder_auto_mover
  LIBRARY DESTINATION lib
)

########## Install worlds + launch ##########
install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

########## Done ##########
ament_package()
