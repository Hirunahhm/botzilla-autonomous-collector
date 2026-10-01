cmake_minimum_required(VERSION 3.8)
project(botzilla_coverage_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(botzilla_coverage_layer_plugin SHARED
  src/coverage_cost_layer.cpp)
target_include_directories(botzilla_coverage_layer_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(botzilla_coverage_layer_plugin PUBLIC cxx_std_17)

ament_target_dependencies(botzilla_coverage_layer_plugin
  rclcpp
  nav2_costmap_2d
  nav_msgs
  pluginlib)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugin.xml)

install(TARGETS botzilla_coverage_layer_plugin
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
ament_export_libraries(botzilla_coverage_layer_plugin)
ament_export_dependencies(rclcpp nav2_costmap_2d nav_msgs pluginlib)

ament_package()
