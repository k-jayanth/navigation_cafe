cmake_minimum_required(VERSION 3.8)
project(robot_behaviour_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(robot_interfaces REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  nav2_msgs
  nav2_behavior_tree
  nav2_util
  rclcpp_components
  robot_interfaces
)

# Action Nodes
add_library(NavigateToTargetAction_bt_node SHARED src/navigate_to_target.cpp)
list(APPEND plugin_libs NavigateToTargetAction_bt_node)

add_library(robot_behaviour_tree SHARED src/go_to_target.cpp )
rclcpp_components_register_node(robot_behaviour_tree PLUGIN "GoToTargetActionServer" EXECUTABLE go_to_target_action_server)

ament_target_dependencies(robot_behaviour_tree ${dependencies})

# Install Plugins
foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin}
    ${dependencies}
  )
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(
  TARGETS robot_behaviour_tree ${plugin_libs}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(
  DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})


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

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${plugin_libs}
  ${debug_nodes}
)

ament_export_dependencies(
  ${dependencies}
)

ament_package()
