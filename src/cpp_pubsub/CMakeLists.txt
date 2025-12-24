cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geodesy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(ps4 src/ps4_teleop.cpp)
ament_target_dependencies(ps4 rclcpp geometry_msgs sensor_msgs)

add_executable(2ps4 src/ps4_combined.cpp)
ament_target_dependencies(2ps4 rclcpp geometry_msgs sensor_msgs)

add_executable(gps_sub src/gps_sub.cpp)
ament_target_dependencies(gps_sub rclcpp geometry_msgs sensor_msgs std_msgs)

add_executable(lidarmin src/lidar_min_dist.cpp)
ament_target_dependencies(lidarmin rclcpp geometry_msgs sensor_msgs)

add_executable(rough src/rough.cpp)
ament_target_dependencies(rough rclcpp geometry_msgs sensor_msgs)

add_executable(rough2 src/ROUGH.cpp)
ament_target_dependencies(rough2 rclcpp geometry_msgs sensor_msgs std_msgs)

add_executable(rough3 src/rough3.cpp)
ament_target_dependencies(rough3 geodesy rclcpp geometry_msgs sensor_msgs std_msgs geographic_msgs)

add_executable(rough4 src/rough4.cpp)
ament_target_dependencies(rough4 geodesy rclcpp geometry_msgs sensor_msgs std_msgs geographic_msgs)

add_executable(rough5 src/rough5.cpp)
ament_target_dependencies(rough5 geodesy rclcpp geometry_msgs sensor_msgs std_msgs geographic_msgs)

add_executable(rough6 src/rough6.cpp)
ament_target_dependencies(rough6 geodesy rclcpp geometry_msgs sensor_msgs std_msgs geographic_msgs)

add_executable(rough7 src/rough7.cpp)
ament_target_dependencies(rough7 geodesy rclcpp geometry_msgs sensor_msgs std_msgs geographic_msgs)


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
install(TARGETS
  ps4
  2ps4
  gps_sub
  lidarmin
  rough
  rough2
  rough3
  rough4
  rough5
  rough6
  rough7
  DESTINATION lib/${PROJECT_NAME})

ament_package()
