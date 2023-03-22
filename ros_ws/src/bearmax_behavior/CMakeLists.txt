cmake_minimum_required(VERSION 3.8)
project(bearmax_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(behavior_tree src/behavior_tree.cpp)
target_include_directories(behavior_tree PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(behavior_tree PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  behavior_tree
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_action"
  "ament_index_cpp"
  "behaviortree_cpp_v3"
)

install(TARGETS behavior_tree
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch trees DESTINATION share/${PROJECT_NAME})

install(
    DIRECTORY include
    DESTINATION include/${PROJECT_NAME}
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
