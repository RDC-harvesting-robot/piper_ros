#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

geometry_msgs::msg::Point latest_target_position;
bool received_target_position = false;

void targetPositionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
  latest_target_position = *msg;
  received_target_position = true;
  RCLCPP_INFO(rclcpp::get_logger("demo_arm_control"), "Received: [%.3f, %.3f, %.3f]",
              latest_target_position.x, latest_target_position.y, latest_target_position.z);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // ノードの作成
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides({{"use_sim_time", true}});
  auto node = std::make_shared<rclcpp::Node>("demo_arm_control", node_options);

  // 別スレッドでspinを開始
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spinner_thread([executor]() { executor->spin(); });

  // MoveGroupInterface 初期化
  moveit::planning_interface::MoveGroupInterface move_group_arm(node, "arm");
  moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "gripper");
  // 現在のポーズを取得して目標位置をセット
  geometry_msgs::msg::Pose target_pose = move_group_arm.getCurrentPose().pose;

  // トピックのサブスクライブ設定
  auto subscription = node->create_subscription<geometry_msgs::msg::Point>(
      "target_position", 10, targetPositionCallback);

  // --------------------------------arm control-------------------------------------

  move_group_arm.setNamedTarget("zero");
  move_group_arm.move();
  move_group_gripper.setNamedTarget("open");
  move_group_gripper.move();
  move_group_arm.setNamedTarget("set");
  move_group_arm.move();

  // 目標位置を受信するまで待機
  target_pose =  move_group_arm.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Waiting for target position...");
  while (rclcpp::ok() && !received_target_position) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    target_pose.position.y = 0.1;
    move_group_arm.setPoseTarget(target_pose);
    move_group_arm.move();
    target_pose.position.y = -0.1;
    move_group_arm.setPoseTarget(target_pose);
    move_group_arm.move();
  }
  target_pose = move_group_arm.getCurrentPose().pose;
  target_pose.position.x = latest_target_position.x;
  target_pose.position.y = latest_target_position.y;
  target_pose.position.z = latest_target_position.z;

  move_group_arm.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_arm.execute(plan);
  } else {
    RCLCPP_WARN(node->get_logger(), "Planning failed.");
  }

  move_group_gripper.setNamedTarget("close");
  move_group_gripper.move();

  move_group_arm.setNamedTarget("set");
  move_group_arm.move();

  // --------------------------------arm control-------------------------------------

  // 終了処理
  executor->cancel();
  spinner_thread.join();

  rclcpp::shutdown();
  return 0;
}
