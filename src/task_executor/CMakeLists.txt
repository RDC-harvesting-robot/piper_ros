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



# 実行ファイルのビルド
add_executable(demo_arm_control src/demo_arm_control.cpp)
ament_target_dependencies(
  demo_arm_control
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
)

add_executable(ee_demo_arm_control src/ee_demo_arm_control.cpp)
ament_target_dependencies(
  ee_demo_arm_control
  example_interfaces
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
)

add_executable(pose_set_client src/pose_set_client.cpp)
ament_target_dependencies(pose_set_client rclcpp example_interfaces)
install(TARGETS pose_set_client DESTINATION lib/${PROJECT_NAME})

add_executable(target_position_pub src/target_position_pub.cpp)
ament_target_dependencies(
  target_position_pub
  rclcpp
  geometry_msgs
)

add_executable(tf_broadcast src/tf_broadcast.cpp)
ament_target_dependencies(tf_broadcast
  rclcpp
  std_msgs
  tf2_ros
  geometry_msgs
)

add_executable(tf_arm_control src/tf_arm_control.cpp)
ament_target_dependencies(
  tf_arm_control
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)


install(TARGETS
  tf_broadcast
  DESTINATION lib/${PROJECT_NAME}
)

# インストール設定
install(TARGETS
  demo_arm_control
  ee_demo_arm_control
  target_position_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  tf_arm_control
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
