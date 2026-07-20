cmake_minimum_required(VERSION 3.20)
project(opennav_following)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(opennav_docking REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

# NOTE: can't be named exactly "opennav_following" — rosidl_generate_interfaces()
# below already claims that name for its own custom target within this project.
set(executable_name opennav_following_node)
set(library_name opennav_following_core)

# This package ships its own FollowObject action instead of depending on
# nav2_msgs, so it installs standalone on Jazzy without needing a patched
# nav2_msgs overlay. See README.md for why (upstream's nav2_msgs does not
# have this action on the Jazzy release line).
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowObject.action"
  DEPENDENCIES builtin_interfaces
)

set(dependencies
  angles
  geometry_msgs
  nav2_util
  opennav_docking
  opennav_docking_core
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(${library_name} SHARED
  src/following_server.cpp
)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(${library_name} PUBLIC ${dependencies})

# Link against the action types generated above (FollowObject) from this
# same package.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${library_name} PUBLIC "${cpp_typesupport_target}")

add_executable(${executable_name}
  src/main.cpp
)
target_include_directories(${executable_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${executable_name} PRIVATE ${library_name})
ament_target_dependencies(${executable_name} PUBLIC rclcpp)

rclcpp_components_register_nodes(${library_name} "opennav_following::FollowingServer")

install(TARGETS ${library_name}
  EXPORT export_${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_lint_auto_find_test_dependencies()
  ament_find_gtest()
  add_subdirectory(test)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  action_msgs
  builtin_interfaces
  geometry_msgs
  nav2_util
  opennav_docking
  opennav_docking_core
  rclcpp
  rclcpp_lifecycle
  rosidl_default_runtime
  tf2_ros
)
ament_export_targets(export_${library_name})
ament_package()
