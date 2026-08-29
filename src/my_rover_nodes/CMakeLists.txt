cmake_minimum_required(VERSION 3.8)
project(my_rover_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

find_package(my_rover_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(minimal_publisher src/minimal_publisher.cpp)
ament_target_dependencies(minimal_publisher rclcpp std_msgs)

add_executable(minimal_subscriber src/minimal_subscriber.cpp)
ament_target_dependencies(minimal_subscriber rclcpp std_msgs)

add_executable(minimal_server src/minimal_server.cpp)
ament_target_dependencies(minimal_server rclcpp example_interfaces)

add_executable(minimal_requester src/minimal_requester.cpp)
ament_target_dependencies(minimal_requester rclcpp example_interfaces)

add_executable(countdown_server src/countdown_server.cpp)
ament_target_dependencies(countdown_server rclcpp rclcpp_action my_rover_interfaces)

add_executable(countdown_client src/countdown_client.cpp)
ament_target_dependencies(countdown_client rclcpp rclcpp_action my_rover_interfaces)

add_executable(system_mode_manager src/system_mode_manager.cpp)
ament_target_dependencies(system_mode_manager rclcpp rclcpp_action std_msgs nav2_msgs my_rover_interfaces)

add_executable(safety_monitor src/safety_monitor.cpp)
ament_target_dependencies(safety_monitor rclcpp rclcpp_action std_srvs std_msgs nav2_msgs sensor_msgs nav_msgs my_rover_interfaces)

install(TARGETS
  minimal_publisher
  minimal_subscriber
  minimal_server
  minimal_requester
  countdown_server       
  countdown_client
  system_mode_manager
  safety_monitor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

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
