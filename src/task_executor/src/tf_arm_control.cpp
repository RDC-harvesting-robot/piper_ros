#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // ノード作成 & executor
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({{"use_sim_time", true}});
  auto node = std::make_shared<rclcpp::Node>("tf_arm_control", node_options);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spinner_thread([&executor]() { executor->spin(); });

  // tf2準備
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  std::string target_frame = "target_object";
  std::string reference_frame = "base_link";

  // MoveGroupインターフェースの初期化
  moveit::planning_interface::MoveGroupInterface move_group_arm(node, "arm");
  moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "gripper");

  // === 初期ポーズ移動 ===
  move_group_arm.setNamedTarget("zero");
  move_group_arm.move();

  move_group_gripper.setNamedTarget("open");
  move_group_gripper.move();

  move_group_arm.setNamedTarget("hr_demo_set");
  move_group_arm.move();

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  // === tfからtarget_objectの位置を取得 ===
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

  RCLCPP_INFO(node->get_logger(), "Target position: x=%.3f y=%.3f z=%.3f",
              transformStamped.transform.translation.x,
              transformStamped.transform.translation.y,
              transformStamped.transform.translation.z);

  // === アプローチ動作 ===
  geometry_msgs::msg::Pose target_pose = move_group_arm.getCurrentPose().pose;

  target_pose.position.x = transformStamped.transform.translation.x;
  target_pose.position.y = transformStamped.transform.translation.y;
  target_pose.position.z = transformStamped.transform.translation.z;

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
    executor->cancel();
    spinner_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  // === 把持動作 ===
  move_group_gripper.setNamedTarget("close");
  move_group_gripper.move();

  // === 復帰動作 ===
  move_group_arm.setNamedTarget("hr_demo_set");
  move_group_arm.move();

  move_group_arm.setNamedTarget("zero");
  move_group_arm.move();

  // === 終了処理 ===
  executor->cancel();
  spinner_thread.join();
  rclcpp::shutdown();
  return 0;
}
