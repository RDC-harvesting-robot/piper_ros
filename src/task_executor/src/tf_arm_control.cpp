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


  
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // --- 移動① (ZY平面での位置合わせ) ---
  RCLCPP_INFO(node->get_logger(), "Executing Step 1: ZY Alignment");
  geometry_msgs::msg::Pose step1_pose = move_group_arm.getCurrentPose().pose;
  
  // 目標のY座標とZ座標に合わせる (X座標は現在のまま)
  step1_pose.position.y = transformStamped.transform.translation.y;
  step1_pose.position.z = transformStamped.transform.translation.z;
  
  move_group_arm.setPoseTarget(step1_pose);
  if (move_group_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group_arm.execute(plan);
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Planning failed for Step 1.");
  }

  rclcpp::sleep_for(std::chrono::milliseconds(500)); // 安定待ち

  // --- 移動② (X方向へのアプローチ) ---
  RCLCPP_INFO(node->get_logger(), "Executing Step 2: X Approach");
  geometry_msgs::msg::Pose step2_pose = move_group_arm.getCurrentPose().pose;

  // 目標のX座標に合わせる (YとZ座標は現在のまま)
  /
  step2_pose.position.x = transformStamped.transform.translation.x - 0.15;

  move_group_arm.setPoseTarget(step2_pose);
  if (move_group_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group_arm.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Approach succeeded.");
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Planning failed for Step 2.");
  }



  rclcpp::sleep_for(std::chrono::seconds(1));

  // === 把持（必要に応じて） ===
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