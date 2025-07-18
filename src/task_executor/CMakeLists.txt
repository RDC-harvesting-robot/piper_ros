cmake_minimum_required(VERSION 3.8)
project(task_executor)

# コンパイルオプション
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#================================================================
# 1. パッケージ検索
#================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

#================================================================
# 2. 実行可能ファイル定義
#================================================================
add_executable(arm_control_final src/arm_control_final.cpp)
add_executable(demo_arm_control src/demo_arm_control.cpp)
# add_executable(exp_demo_arm_control src/demo_arm_control.cpp) # demo_arm_controlと同じソースのためコメントアウト
add_executable(ur5e_demo_arm_control src/ur5e_demo_arm_control.cpp)
add_executable(ee_demo_arm_control src/ee_demo_arm_control.cpp)
add_executable(pose_set_client src/pose_set_client.cpp)
add_executable(target_position_pub src/target_position_pub.cpp)
add_executable(tf_broadcast src/tf_broadcast.cpp)
add_executable(tf_arm_control src/tf_arm_control.cpp)

#================================================================
# 3. 依存関係
#================================================================
# MoveItを使うコントロール系の共通依存関係をリスト化
set(MOVEIT_CONTROL_DEPS
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  geometry_msgs
)

ament_target_dependencies(arm_control_final rclcpp moveit_ros_planning_interface std_msgs)
ament_target_dependencies(demo_arm_control ${MOVEIT_CONTROL_DEPS})
ament_target_dependencies(ur5e_demo_arm_control ${MOVEIT_CONTROL_DEPS})
ament_target_dependencies(ee_demo_arm_control ${MOVEIT_CONTROL_DEPS} example_interfaces)
ament_target_dependencies(pose_set_client rclcpp example_interfaces)
ament_target_dependencies(target_position_pub rclcpp geometry_msgs)
ament_target_dependencies(tf_broadcast rclcpp std_msgs tf2_ros geometry_msgs)
ament_target_dependencies(tf_arm_control ${MOVEIT_CONTROL_DEPS} tf2 tf2_ros tf2_geometry_msgs)


#================================================================
# 4. インストール
#================================================================
install(
  TARGETS
    arm_control_final
    demo_arm_control
    ur5e_demo_arm_control
    ee_demo_arm_control
    pose_set_client
    target_position_pub
    tf_broadcast
    tf_arm_control
  DESTINATION lib/${PROJECT_NAME}
)

#================================================================
# 5. パッケージ化
#================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()