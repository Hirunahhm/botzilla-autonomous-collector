cmake_minimum_required(VERSION 3.8)
project(botzilla_straightline_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

add_library(botzilla_straightline_planner_plugin SHARED
  src/straight_line_planner.cpp)
target_include_directories(botzilla_straightline_planner_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(botzilla_straightline_planner_plugin PUBLIC cxx_std_17)

ament_target_dependencies(botzilla_straightline_planner_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  pluginlib)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS botzilla_straightline_planner_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(botzilla_straightline_planner_plugin)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d nav2_util
  nav_msgs geometry_msgs tf2 tf2_ros pluginlib)

ament_package()
