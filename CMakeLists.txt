cmake_minimum_required(VERSION 3.8)
project(nav2_proximity_wait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
set(dependencies
  rclcpp
  nav2_behavior_tree
  behaviortree_cpp_v3
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_util
)

find_package(ament_cmake REQUIRED)
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

# ── Plugin shared library ────────────────────────────────────────────────────
add_library(nav2_proximity_wait_bt_node SHARED
  src/check_robot_proximity.cpp
)

target_include_directories(nav2_proximity_wait_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_proximity_wait_bt_node ${dependencies})

# Required so BT.CPP can load the plugin via dlopen
target_compile_definitions(nav2_proximity_wait_bt_node PRIVATE BT_PLUGIN_EXPORT)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS nav2_proximity_wait_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY bt_xml/
  DESTINATION share/${PROJECT_NAME}/bt_xml/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# ── Exports ───────────────────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(nav2_proximity_wait_bt_node)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
