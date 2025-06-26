#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tf_arm_control_node");

  // TF listener setup
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // MoveIt interfaces
  moveit::planning_interface::MoveGroupInterface move_group_arm(node, "arm");
  moveit::planning_interface::MoveGroupInterface move_group_gripper(node, "gripper");

  // 初期姿勢へ移動
  move_group_arm.setNamedTarget("zero");
  move_group_arm.move();
  move_group_gripper.setNamedTarget("open");
  move_group_gripper.move();
  move_group_arm.setNamedTarget("hr_demo_set");
  move_group_arm.move();

  // TF取得待ち
  geometry_msgs::msg::TransformStamped transformStamped;
  while (rclcpp::ok()) {
    try {
      transformStamped = tf_buffer->lookupTransform(
        "base_link",       // ターゲット座標系
        "target_object",   // ソース座標系（カメラから見た物体）
        tf2::TimePointZero
      );
      break;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(node->get_logger(), "Waiting for TF target_object -> base_link: %s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // しきい値チェック (上限1.0m, 下限0.01m)
  double x = transformStamped.transform.translation.x;
  double y = transformStamped.transform.translation.y;
  double z = transformStamped.transform.translation.z;

  if ((std::abs(x) > 1.0 || std::abs(y) > 1.0 || std::abs(z) > 1.0) ||
      (std::abs(x) < 0.01 && std::abs(y) < 0.01 && std::abs(z) < 0.01)) {
    RCLCPP_ERROR(node->get_logger(), "Target out of threshold: x=%.3f y=%.3f z=%.3f", x, y, z);
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Target (base_link): x=%.3f y=%.3f z=%.3f", x, y, z);

  // 目標位置Poseを作成
    geometry_msgs::msg::Pose target_pose = move_group_arm.getCurrentPose().pose; 
    target_pose.position.x = transformStamped.transform.translation.x;
    target_pose.position.y = transformStamped.transform.translation.y;
    target_pose.position.z = transformStamped.transform.translation.z;
  // orientation はそのまま

  // 目標座標にアプローチ
  move_group_arm.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group_arm.execute(plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed to target_object");
  }

  // 把持後の処理
  move_group_gripper.setNamedTarget("close");
  move_group_gripper.move();
  move_group_arm.setNamedTarget("hr_demo_set");
  move_group_arm.move();
  move_group_arm.setNamedTarget("zero");
  move_group_arm.move();

  rclcpp::shutdown();
  return 0;
}
