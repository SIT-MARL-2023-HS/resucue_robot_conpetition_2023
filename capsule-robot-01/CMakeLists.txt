
cmake_minimum_required(VERSION 3.8)
project(capsule-robot-01)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(pigpiod_if2 REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(capsulerobot01 src/capsule-robot-01.cpp)
ament_target_dependencies(capsulerobot01 rclcpp sensor_msgs)

target_compile_options(capsulerobot01
  PUBLIC -Wall -pthread
)
target_link_libraries(capsulerobot01
  pigpiod_if2
)

install(TARGETS
  capsulerobot01
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
