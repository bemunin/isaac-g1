cmake_minimum_required(VERSION 3.10)
project(oc_foxglove_bridge NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes urdf DESTINATION share/${PROJECT_NAME})

ament_package()
