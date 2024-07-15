cmake_minimum_required(VERSION 3.8)
project(bt_leaf_omron)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake            REQUIRED)
find_package(rclcpp                 REQUIRED)
find_package(behaviortree_ros2      REQUIRED)
find_package(btcpp_ros2_interfaces  REQUIRED)
find_package(omron_msgs             REQUIRED)
find_package(cnr_param             REQUIRED)


add_library(${PROJECT_NAME}_plugin SHARED
  include/${PROJECT_NAME}/go_to_goal.hpp
  src/${PROJECT_NAME}/go_to_goal.cpp)

ament_target_dependencies(${PROJECT_NAME}_plugin PUBLIC
  rclcpp
  behaviortree_ros2
  btcpp_ros2_interfaces
  omron_msgs
  cnr_param)

target_compile_definitions(${PROJECT_NAME}_plugin PRIVATE  BT_PLUGIN_EXPORT)

target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS ${PROJECT_NAME}_plugin
        EXPORT export_${PROJECT_NAME}_plugin
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME}_plugin)
ament_export_dependencies(
  rclcpp
  behaviortree_ros2
  btcpp_ros2_interfaces
  omron_msgs
  cnr_param)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
