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
find_package(cnr_param              REQUIRED)
find_package(cnr_yaml               REQUIRED)
find_package(geometry_msgs          REQUIRED)


add_library(${PROJECT_NAME}_plugin SHARED
  include/${PROJECT_NAME}/go_to_goal.hpp
  src/${PROJECT_NAME}/go_to_goal.cpp)
list(APPEND plugin_libs ${PROJECT_NAME}_plugin)

add_library(navigate_to_pose SHARED 
  include/${PROJECT_NAME}/navigate_to_pose.hpp
  src/${PROJECT_NAME}/navigate_to_pose.cpp)
  
list(APPEND plugin_libs navigate_to_pose)

set(dependencies
  rclcpp
  behaviortree_ros2
  btcpp_ros2_interfaces
  omron_msgs
  cnr_param
  cnr_yaml
  geometry_msgs
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)

  target_include_directories(${bt_plugin} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
endforeach()

install(DIRECTORY include
        DESTINATION include/)


install(TARGETS 
        ${plugin_libs}
        EXPORT export_${PROJECT_NAME}_plugin
        RUNTIME DESTINATION lib
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME}_plugin)
ament_export_dependencies(${dependencies})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
