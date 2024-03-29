cmake_minimum_required(VERSION 3.8)
project(bearmax_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bearmax_msgs REQUIRED)

add_executable(task_server src/task_server.cpp src/face_follower_task.cpp)
add_executable(face_follower src/face_follower.cpp)
target_include_directories(task_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(task_server PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(
  task_server
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "bearmax_msgs"
)

target_include_directories(face_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(face_follower PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  face_follower
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "std_srvs"
  "geometry_msgs"
  "bearmax_msgs"
)

install(TARGETS task_server face_follower
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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
