cmake_minimum_required(VERSION 3.10)
project(oc_sonic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(oc_sonic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(onnxruntime CONFIG REQUIRED)

add_library(oc_sonic SHARED
  src/sonic_common.cpp
  src/motion.cpp
  src/sonic_policy.cpp
  src/kinematic_planner.cpp
  src/planner_command.cpp
  src/planner_node.cpp
  src/policy_node.cpp
)
target_include_directories(oc_sonic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(oc_sonic rclcpp rclcpp_components geometry_msgs sensor_msgs std_msgs oc_sonic_msgs)
target_link_libraries(oc_sonic Eigen3::Eigen onnxruntime::onnxruntime)
rclcpp_components_register_nodes(oc_sonic "oc_sonic::PlannerNode" "oc_sonic::PolicyNode")

install(TARGETS oc_sonic ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_golden test/test_golden.cpp)
  target_link_libraries(test_golden oc_sonic)
  target_compile_definitions(test_golden PRIVATE
    GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
    MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/sonic"
  )
endif()

ament_package()
