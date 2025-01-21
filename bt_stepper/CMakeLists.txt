cmake_minimum_required(VERSION 3.8)
project(bt_stepper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(
  include
  ${behaviortree_cpp_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

set(dependencies
  behaviortree_cpp
  rclcpp
)

add_executable(bt_stepper
  src/main.cpp
)
ament_target_dependencies(bt_stepper
  ${dependencies}
)
target_link_libraries(bt_stepper
)

add_executable(bt_run
  src/run.cpp
)
ament_target_dependencies(bt_run
  ${dependencies}
)
target_link_libraries(bt_run
)
install(TARGETS bt_stepper bt_run
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
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
