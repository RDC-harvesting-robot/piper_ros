cmake_minimum_required(VERSION 3.8)
project(task_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 必要なパッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# 実行ファイルのビルド
add_executable(target_position_pub src/target_position_pub.cpp)
ament_target_dependencies(
  target_position_pub
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  yaml-cpp
)

add_executable(tf_filter_node src/tf_filter_node.cpp)
ament_target_dependencies(
  tf_filter_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

add_executable(ur5e_tf_arm_control src/ur5e_tf_arm_control.cpp)
ament_target_dependencies(
  ur5e_tf_arm_control
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  yaml-cpp
)

target_link_libraries(ur5e_tf_arm_control yaml-cpp)
target_link_libraries(target_position_pub yaml-cpp)

# インストール設定
install(TARGETS
  target_position_pub
  tf_filter_node
  ur5e_tf_arm_control
  DESTINATION lib/${PROJECT_NAME}
)


# テスト設定（必要なら）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()