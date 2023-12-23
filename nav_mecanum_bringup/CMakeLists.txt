cmake_minimum_required(VERSION 3.8)
project(nav_mecanum_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(nav_mecanum_kernel src/nav_mecanum_kernel.cpp)
add_executable(imu_checker src/imu_checker.cpp)

ament_target_dependencies(nav_mecanum_kernel rclcpp sensor_msgs nav_msgs control_msgs)
ament_target_dependencies(imu_checker rclcpp sensor_msgs tf2_msgs geometry_msgs tf2_ros)

install(TARGETS
  nav_mecanum_kernel
  imu_checker
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY config launch
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
