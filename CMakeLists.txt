cmake_minimum_required(VERSION 3.8)
project(autonomous_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(SDL2 REQUIRED)

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

include_directories(include ${SDL2_INCLUDE_DIRS})

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  map_msgs
  visualization_msgs
  tf2
  tf2_ros
)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop rclcpp geometry_msgs)
target_link_libraries(keyboard_teleop ${SDL2_LIBRARIES})

add_executable(autonomous_explorer src/autonomous_explorer.cpp src/frontier_search.cpp)
ament_target_dependencies(autonomous_explorer ${dependencies})

install(TARGETS 
  keyboard_teleop
  autonomous_explorer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  src 
  launch 
  rviz 
  models 
  config 
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
