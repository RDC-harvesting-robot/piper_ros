#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

// 目標のプランニングを関数化
moveit::core::MoveItErrorCode planToTargetPose(moveit::planning_interface::MoveGroupInterface& move_group,
                                              const geometry_msgs::msg::Pose& target_pose,
                                              moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                              rclcpp::Logger logger)
  {
    move_group.setPlanningTime(10.0); // 計算時間の調整
    // move_group.setGoalOrientationTolerance(0.1);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.allowReplanning(true); // 動作計画の再計算を許可
    move_group.setPoseTarget(target_pose);

    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      move_group.execute(plan);
      RCLCPP_INFO(logger, "Plan and execution succeeded.");
    }
    else
    {
      RCLCPP_WARN(logger, "Planning failed.");
    }

    return move_group.plan(plan);
  }

moveit::core::MoveItErrorCode planToTargetPoseCartesian(moveit::planning_interface::MoveGroupInterface& move_group,
                                                        const geometry_msgs::msg::Pose& target_pose,
                                                        moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                                        rclcpp::Logger logger)
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  const double eef_step = 0.01;
  const double jump_threshold = 5.0;
  const double velocity_scale = 0.07;
  const double acceleration_scale = 1.0;

  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(logger, "fraction: %f", fraction);

  if (fraction > 0.70)
  {
    robot_trajectory::RobotTrajectory rt_step(move_group.getRobotModel(), move_group.getName());
    rt_step.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);
    rt_step.setGroupName(move_group.getName());

    trajectory_processing::IterativeParabolicTimeParameterization time_param_step;
    bool success_retime = time_param_step.computeTimeStamps(rt_step, velocity_scale, acceleration_scale);
    if (!success_retime)
    {
      RCLCPP_ERROR(logger, "Failed to re-time trajectory.");
      return moveit::core::MoveItErrorCode::PLANNING_FAILED;
    }

    rt_step.getRobotTrajectoryMsg(plan.trajectory_);
    if (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "Cartesian path planning and execution succeeded.");
      return moveit::core::MoveItErrorCode::SUCCESS;
    }
    else
    {
      RCLCPP_ERROR(logger, "Execution failed.");
      return moveit::core::MoveItErrorCode::CONTROL_FAILED;
    }
  }
  else
  {
    RCLCPP_WARN(logger, "Cartesian path planning failed. Success fraction: %.2f", fraction);
    return moveit::core::MoveItErrorCode::PLANNING_FAILED;
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ur5e_tf_arm_control");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  std::thread spinner_thread([&executor]() { executor->spin(); });

  // === tf listener 初期化 ===
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  std::string target_frame = "peduncle"; // アーム先端を到達した目標座標
  std::string reference_frame = "base_link"; // アームの付け根
  std::string endeffector_frame = "endeffector_cutter"; // アーム先端の座標

  // === MoveGroupInterface 初期化 ===
  moveit::planning_interface::MoveGroupInterface move_group_arm(node, "ur_manipulator");
  geometry_msgs::msg::TransformStamped tf_target_to_base; // 目標TF
  geometry_msgs::msg::TransformStamped tf_ee_to_base; // 目標TF
  geometry_msgs::msg::Pose target_pose; // 目標TF座標
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  tf2::Vector3 peduncle_pos_base;
  tf2::Vector3 ee_pos_base; 
  tf2::Vector3 relative_base;
  tf2::Vector3 local_offset;
  tf2::Vector3 world_offset;
  tf2::Vector3 base_target_position;

  // === 初期ポーズへ移動 ===
  // move_group_arm.setNamedTarget("harvest_set");
  // move_group_arm.move();
  // move_group_arm.setNamedTarget("servo_set");
  // move_group_arm.move();

  rclcpp::sleep_for(std::chrono::seconds(1));  // 安定待ち


  // === TFからtarget_object位置取得 ===
  while (rclcpp::ok())
  {
    try
    {
      // ワンショットで果柄（の付け根）の座標を取得
      tf_target_to_base = tf_buffer->lookupTransform(reference_frame, target_frame, tf2::TimePointZero);
      // 果柄の位置を記録
      YAML::Node node_out;
      node_out["position"]["x"] = tf_target_to_base.transform.translation.x;
      node_out["position"]["y"] = tf_target_to_base.transform.translation.y;
      node_out["position"]["z"] = tf_target_to_base.transform.translation.z;
      node_out["orientation"]["x"] = tf_target_to_base.transform.rotation.x;
      node_out["orientation"]["y"] = tf_target_to_base.transform.rotation.y;
      node_out["orientation"]["z"] = tf_target_to_base.transform.rotation.z;
      node_out["orientation"]["w"] = tf_target_to_base.transform.rotation.w;
      std::string yaml_path = "/home/yugonishio/ros2_ws/src/piper_ros/src/task_executor/config/last_tf_pose.yaml";
      std::ofstream ofs(yaml_path);
      ofs << node_out; // yamlファイルに書き込み
      break;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(node->get_logger(), "Waiting for TF target_object: %s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
  }

  tf2::Transform T_target, T_ee;
  tf2::fromMsg(tf_target_to_base.transform, T_target);
  peduncle_pos_base = T_target.getOrigin(); // base_linkからpeduncleの絶対位置を記録しておく．


  // === アプローチ姿勢 ===
  target_pose.position = move_group_arm.getCurrentPose().pose.position; // 手先の位置は今の位置
  target_pose.orientation = tf_target_to_base.transform.rotation; // 果柄に向けて姿勢を変更
  // planToTargetPose(move_group_arm, target_pose, plan, node->get_logger());
  planToTargetPoseCartesian(move_group_arm, target_pose, plan, node->get_logger());

  rclcpp::sleep_for(std::chrono::seconds(1));  // 安定待ち

  // <<<<<<<アプローチ位置に移動>>>>>>>>> 
  // 果柄（peduncle）とendeffectorの相対位置をbase_linkからの絶対座標で求める
  tf_ee_to_base = tf_buffer->lookupTransform(reference_frame, endeffector_frame, tf2::TimePointZero);
  tf2::fromMsg(tf_ee_to_base.transform, T_ee);
  ee_pos_base = T_ee.getOrigin();
  relative_base = peduncle_pos_base - ee_pos_base;
  tf2::Matrix3x3 R_base_to_ee = T_ee.getBasis().inverse();  // 回転行列の逆（base→ee）
  tf2::Vector3 relative_local = R_base_to_ee * relative_base;
  local_offset = tf2::Vector3(relative_local.x(), relative_local.y(), 0.0);
  world_offset = T_ee.getBasis() * local_offset;
  // <<<<<<<アプローチ位置に移動>>>>>>>>> 
  target_pose.orientation = move_group_arm.getCurrentPose().pose.orientation; // 手先の姿勢を維持
  target_pose.position.x = move_group_arm.getCurrentPose().pose.position.x + world_offset.x(); // 手先の姿勢に従って移動
  target_pose.position.y = move_group_arm.getCurrentPose().pose.position.y + world_offset.y(); // 手先の姿勢に従って移動
  target_pose.position.z = move_group_arm.getCurrentPose().pose.position.z + world_offset.z();
  // planToTargetPose(move_group_arm, target_pose, plan, node->get_logger());
  planToTargetPoseCartesian(move_group_arm, target_pose, plan, node->get_logger());

  rclcpp::sleep_for(std::chrono::seconds(1));  // 安定待ち

  // <<<<<<<アプローチ動作>>>>>>>>> 
  local_offset = tf2::Vector3(-0.03, 0.0, relative_local.z());  // 一番初めに見た瞬間の相対位置z
  world_offset = T_ee.getBasis() * local_offset;
  // <<<<<<<アプローチ動作>>>>>>>>> 
  target_pose.orientation = move_group_arm.getCurrentPose().pose.orientation; // 手先の姿勢を維持
  target_pose.position = move_group_arm.getCurrentPose().pose.position;
  target_pose.position.x = move_group_arm.getCurrentPose().pose.position.x + world_offset.x(); // 果柄を引っ掛ける分横にずれる
  target_pose.position.y = move_group_arm.getCurrentPose().pose.position.y + world_offset.y();
  target_pose.position.z = move_group_arm.getCurrentPose().pose.position.z + world_offset.z(); // 奥にアプローチ
  // planToTargetPose(move_group_arm, target_pose, plan, node->get_logger());
  planToTargetPoseCartesian(move_group_arm, target_pose, plan, node->get_logger());

  rclcpp::sleep_for(std::chrono::seconds(1));  // 安定待ち

  // <<<<<<<引掛け動作>>>>>>>>>
  local_offset = tf2::Vector3(0.04, 0.0, 0.0);
  world_offset = T_ee.getBasis() * local_offset;
  // <<<<<<<引掛け動作>>>>>>>>>
  target_pose.orientation = move_group_arm.getCurrentPose().pose.orientation; // 手先の姿勢を維持
  target_pose.position = move_group_arm.getCurrentPose().pose.position;
  target_pose.position.x = move_group_arm.getCurrentPose().pose.position.x + world_offset.x(); // 果柄を引っ掛ける分横にずれる
  target_pose.position.y = move_group_arm.getCurrentPose().pose.position.y + world_offset.y();
  target_pose.position.z = move_group_arm.getCurrentPose().pose.position.z + world_offset.z();
  // planToTargetPose(move_group_arm, target_pose, plan, node->get_logger());
  planToTargetPoseCartesian(move_group_arm, target_pose, plan, node->get_logger());


  // ↓　上下左右に移動して奥にアプローチする動作．果柄の位置のみ着目

  //   // === アプローチ位置移動 ===
  // target_pose.orientation = move_group_arm.getCurrentPose().pose.orientation; // 手先の姿勢を維持
  // target_pose.position.x = tf_target_to_base.transform.translation.x;
  // target_pose.position.y = move_group_arm.getCurrentPose().pose.position.y; // まだ奥方向には行かない
  // target_pose.position.z = tf_target_to_base.transform.translation.z;
  // planToTargetPose(move_group_arm, target_pose, plan, node->get_logger());

  //   // === まっすぐ動作 ===
  // target_pose.orientation = move_group_arm.getCurrentPose().pose.orientation; // 手先の姿勢を維持
  // target_pose.position.x = tf_target_to_base.transform.translation.x + 0.03;
  // target_pose.position.y = tf_target_to_base.transform.translation.y;
  // target_pose.position.z = tf_target_to_base.transform.translation.z;
  // planToTargetPose(move_group_arm, target_pose, plan, node->get_logger());

  // // === 引掛け動作 ===
  // target_pose.orientation = move_group_arm.getCurrentPose().pose.orientation; // 手先の姿勢を維持
  // target_pose.position.x = tf_target_to_base.transform.translation.x - 0.04;
  // target_pose.position.y = tf_target_to_base.transform.translation.y; 
  // target_pose.position.z = tf_target_to_base.transform.translation.z;
  // planToTargetPose(move_group_arm, target_pose, plan, node->get_logger());


  rclcpp::sleep_for(std::chrono::seconds(1));

  // === 初期ポーズへ移動 ===
  // move_group_arm.setNamedTarget("servo_set");
  // move_group_arm.move();
  // move_group_arm.setNamedTarget("harvest_set");
  // move_group_arm.move();

  
  // === 終了処理 ===
  executor->cancel();
  spinner_thread.join();
  rclcpp::shutdown();
  return 0;
}
