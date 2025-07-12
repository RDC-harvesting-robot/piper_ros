#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("tf_arm_control");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spinner_thread([&executor]() { executor->spin(); });

  // === tf listener 初期化 ===
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  std::string target_frame = "target_object";
  std::string reference_frame = "base_link";

  // === MoveGroupInterface 初期化 ===
  moveit::planning_interface::MoveGroupInterface move_group_arm(node, "arm");
  moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "gripper");

  // === 初期ポーズへ移動 ===
  move_group_arm.setNamedTarget("hr_demo_set");
  move_group_arm.move();

  move_group_gripper.setNamedTarget("open");
  move_group_gripper.move();

  rclcpp::sleep_for(std::chrono::seconds(1));  // 安定待ち

  // === camera_link → gripper_base → target_object の静的TFをブロードキャスト ===
  // ※この部分はlaunchファイルやコマンドラインで実行する必要あり。->tf_broadcast.cpp
  // ros2 run tf2_ros static_transform_publisher -0.03 0.02 0 1.5708 -1.5708 0 gripper_base camera_link

  // === TFからtarget_object位置取得 ===
  geometry_msgs::msg::TransformStamped transformStamped;
  while (rclcpp::ok())
  {
    try
    {
      transformStamped = tf_buffer->lookupTransform(
          reference_frame, target_frame, tf2::TimePointZero);
      break;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(node->get_logger(), "Waiting for TF target_object -> base_link: %s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
  }

  // === 目標位置ログ出力 ===
  RCLCPP_INFO(node->get_logger(), "Target position: x=%.3f y=%.3f z=%.3f",
              transformStamped.transform.translation.x,
              transformStamped.transform.translation.y,
              transformStamped.transform.translation.z);

  // === アプローチ姿勢構築 ===
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation = move_group_arm.getCurrentPose().pose.orientation;
  target_pose.position.x = transformStamped.transform.translation.x - 0.28;
  target_pose.position.y = transformStamped.transform.translation.y ;
  target_pose.position.z = transformStamped.transform.translation.z ;

  move_group_arm.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group_arm.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Approach succeeded.");
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Planning to target failed.");
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  // === 把持（必要に応じて） ===
  // 正直これは趣味で追加したものなので適宜変更してください
  move_group_gripper.setNamedTarget("close");
  move_group_gripper.move();

  rclcpp::sleep_for(std::chrono::seconds(1));

  // === 復帰動作 ===
  move_group_arm.setNamedTarget("hr_demo_set");
  move_group_arm.move();

  // === 終了処理 ===
  executor->cancel();
  spinner_thread.join();
  rclcpp::shutdown();
  return 0;
}
