cmake_minimum_required(VERSION 3.5)
project(kr_robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY 
  r6bot/meshes r6bot/srdf r6bot/urdf r6bot/rviz
  DESTINATION share/${PROJECT_NAME}/r6bot
)

install(
  DIRECTORY 
  kr810/meshes kr810/urdf kr810/rviz
  DESTINATION share/${PROJECT_NAME}/kr810
)

ament_package()
