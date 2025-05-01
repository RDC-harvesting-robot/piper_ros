#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

std_msgs::msg::Int32MultiArray latest_target_position;
bool received_target_position = false;
bool target_within_threshold = false;

// === コールバック関数 ===
void targetPositionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  latest_target_position = *msg;

  // ★ここでしきい値判定
  int threshold_target_x = 1000;
  int threshold_target_y = 1000;
  int threshold_target_z = 1000;
  int threshold_target_x_low = 10;
  int threshold_target_y_low = 10;
  int threshold_target_z_low = 10;
  if (std::abs(latest_target_position.data[0]) <= threshold_target_y &&
      std::abs(latest_target_position.data[1]) <= threshold_target_z &&
      std::abs(latest_target_position.data[2]) <= threshold_target_x)
  {
    if (std::abs(latest_target_position.data[0]) >= threshold_target_y_low ||
        std::abs(latest_target_position.data[1]) >= threshold_target_z_low ||
        std::abs(latest_target_position.data[2]) >= threshold_target_x_low){
      target_within_threshold = true;
      RCLCPP_INFO(rclcpp::get_logger("demo_arm_control"), "Accepted target: [%d, %d, %d]",
                  latest_target_position.data[0], latest_target_position.data[1], latest_target_position.data[2]);
    }
  }
  else
  {
    target_within_threshold = false;
    RCLCPP_WARN(rclcpp::get_logger("demo_arm_control"), "Rejected target (out of threshold): [%d, %d, %d]",
                latest_target_position.data[0], latest_target_position.data[1], latest_target_position.data[2]);
  }

  received_target_position = true;
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
  auto subscription = node->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/crop_cordinate", 1, targetPositionCallback);

  // --------------------------------arm control-------------------------------------

  move_group_arm.setNamedTarget("zero");
  move_group_arm.move();
  move_group_gripper.setNamedTarget("open");
  move_group_gripper.move();
  move_group_arm.setNamedTarget("hr_demo_set");
  move_group_arm.move();

  rclcpp::sleep_for(std::chrono::milliseconds(1000));


  // 目標位置を受信するまで待機
  target_pose =  move_group_arm.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Waiting for target position...");
  while (rclcpp::ok() && !received_target_position || !target_within_threshold) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    target_pose =  move_group_arm.getCurrentPose().pose;
    RCLCPP_INFO(node->get_logger(), "Current pose: x=%.3f y=%.3f z=%.3f",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);
  }

  RCLCPP_INFO(rclcpp::get_logger("demo_arm_control"), "BEFORE LATEST TARGET POSITION: [%f, %f, %f]",
  target_pose.position.x , target_pose.position.y, target_pose.position.z);

  target_pose = move_group_arm.getCurrentPose().pose;
  // float target_x = latest_target_position.data[2];
  float target_x = latest_target_position.data[2]*0.001; // 前後
  float target_y = latest_target_position.data[0]*0.001*-1; // 左右
  float target_z = latest_target_position.data[1]*0.001; // 上下
  target_pose.position.x += 0;
  target_pose.position.y += 0;
  target_pose.position.z += target_z;
  RCLCPP_INFO(rclcpp::get_logger("demo_arm_control"), "POSITION: [%f]", target_z);
  RCLCPP_INFO(rclcpp::get_logger("demo_arm_control"), "POSITIONaaaaaaaaaaaaaaa: [%f]", latest_target_position.data[2]);
  move_group_arm.setPoseTarget(target_pose);

  RCLCPP_INFO(rclcpp::get_logger("demo_arm_control"), "AFTOR LATEST TARGET POSITION: [%f, %f, %f]",
  target_pose.position.x , target_pose.position.y, target_pose.position.z);

  RCLCPP_INFO(rclcpp::get_logger("demo_arm_control"), "TRGET POSE !!!!: x=%.3f y=%.3f z=%.3f",
  target_pose.position.x,
  target_pose.position.y,
  target_pose.position.z);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_arm.execute(plan);
  } else {
    RCLCPP_WARN(node->get_logger(), "Planning failed.");
  }

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  target_pose = move_group_arm.getCurrentPose().pose;
  RCLCPP_INFO(rclcpp::get_logger("demo_arm_control"), "BEFORE LATEST TARGET POSITION: [%f, %f, %f]",
  target_pose.position.x , target_pose.position.y, target_pose.position.z);
  target_pose.position.x += target_x;
  target_pose.position.y += 0;
  target_pose.position.z += 0;
  move_group_arm.setPoseTarget(target_pose);
  if (move_group_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_arm.execute(plan);
  } else {
    RCLCPP_WARN(node->get_logger(), "Planning failed.");
  }
  RCLCPP_INFO(rclcpp::get_logger("demo_arm_control"), "AFTOR LATEST TARGET POSITION: [%f, %f, %f]",
  target_pose.position.x , target_pose.position.y, target_pose.position.z);

  move_group_gripper.setNamedTarget("close");
  move_group_gripper.move();

  move_group_arm.setNamedTarget("hr_demo_set");
  move_group_arm.move();
  move_group_arm.setNamedTarget("zero");
  move_group_arm.move();
  // --------------------------------arm control-------------------------------------

  // 終了処理
  executor->cancel();
  spinner_thread.join();

  rclcpp::shutdown();
  return 0;
}