#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <thread>

// グローバル変数とコールバック関数 (変更なし)
std_msgs::msg::Int32MultiArray latest_target_position;
bool received_target_position = false;
bool target_within_threshold = false;

void targetPositionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() != 3) { return; }
    latest_target_position = *msg;
    int threshold_target_x = 1000, threshold_target_y = 1000, threshold_target_z = 1000;
    int threshold_target_x_low = 10, threshold_target_y_low = 10, threshold_target_z_low = 10;
    if (std::abs(latest_target_position.data[0]) <= threshold_target_y &&
        std::abs(latest_target_position.data[1]) <= threshold_target_z &&
        std::abs(latest_target_position.data[2]) <= threshold_target_x) {
        if (std::abs(latest_target_position.data[0]) >= threshold_target_y_low ||
            std::abs(latest_target_position.data[1]) >= threshold_target_z_low ||
            std::abs(latest_target_position.data[2]) >= threshold_target_x_low) {
            target_within_threshold = true;
        }
    } else {
        target_within_threshold = false;
    }
    received_target_position = true;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({{"use_sim_time", true}});
    auto node = std::make_shared<rclcpp::Node>("arm_control_stable_plan", node_options);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    std::thread spinner_thread([executor]() { executor->spin(); });

    {
        moveit::planning_interface::MoveGroupInterface move_group_arm(node, "arm");
        auto subscription = node->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/crop_cordinate", 1, targetPositionCallback);

        // 1. 初期姿勢へ移動
        move_group_arm.setNamedTarget("hr_demo_set");
        move_group_arm.move();
        
        // 2. この時点の向きを「固定する向き」として保存
        const auto fixed_orientation = move_group_arm.getCurrentPose().pose.orientation;
        
        RCLCPP_INFO(node->get_logger(), "Reference orientation set. Waiting for target...");

        // 3. 座標受信を待つ
        while (rclcpp::ok() && (!received_target_position || !target_within_threshold)) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(node->get_logger(), "Target received. Planning motion...");
        
        // 4. 速度を20%に設定
        move_group_arm.setMaxVelocityScalingFactor(0.2);
        
        // --- 定数定義 ---
        const double eef_step = 0.01;
        const double jump_threshold = 5.0;

        // --- 移動 ① (ZY平面での位置合わせ) ---
        RCLCPP_INFO(node->get_logger(), "Planning step 1 (ZY alignment) with Cartesian Path...");
        
        std::vector<geometry_msgs::msg::Pose> waypoints_step1;
        geometry_msgs::msg::Pose target_pose_step1 = move_group_arm.getCurrentPose().pose;

        float target_y_offset = latest_target_position.data[0] * -0.001;
        float target_z_offset = latest_target_position.data[1] * 0.001;
        
        target_pose_step1.position.y += target_y_offset + 0.03;
        target_pose_step1.position.z += target_z_offset + 0.08;
        target_pose_step1.orientation = fixed_orientation; // ★ 向きを明示的に再設定
        waypoints_step1.push_back(target_pose_step1);

        moveit_msgs::msg::RobotTrajectory trajectory_step1;
        double fraction_step1 = move_group_arm.computeCartesianPath(waypoints_step1, eef_step, jump_threshold, trajectory_step1);

        RCLCPP_INFO(node->get_logger(), "Cartesian path plan for step 1 (%.2f%% achieved)", fraction_step1 * 100.0);

        if (fraction_step1 < 0.95) {
            RCLCPP_ERROR(node->get_logger(), "Planning failed for step 1. Aborting.");
            rclcpp::shutdown();
            return -1;
        }
        
        RCLCPP_INFO(node->get_logger(), "Executing step 1 (ZY alignment)...");
        move_group_arm.execute(trajectory_step1);


        // --- 移動 ② (X方向へのアプローチ) ---
        RCLCPP_INFO(node->get_logger(), "Planning step 2 (X approach) with Cartesian Path...");
        
        std::vector<geometry_msgs::msg::Pose> waypoints_step2;
        geometry_msgs::msg::Pose start_pose_step2 = move_group_arm.getCurrentPose().pose;
        start_pose_step2.orientation = fixed_orientation; // ★ 始点の向きも明示的に再設定
        waypoints_step2.push_back(start_pose_step2);
        
        float target_x_offset = latest_target_position.data[2] * 0.001;

        geometry_msgs::msg::Pose final_insert_pose = start_pose_step2;
        final_insert_pose.position.x += target_x_offset - 0.20;
        final_insert_pose.orientation = fixed_orientation; // ★ 終点の向きも明示的に再設定
        waypoints_step2.push_back(final_insert_pose);
        
        moveit_msgs::msg::RobotTrajectory trajectory_step2;
        double fraction_step2 = move_group_arm.computeCartesianPath(waypoints_step2, eef_step, jump_threshold, trajectory_step2);

        RCLCPP_INFO(node->get_logger(), "Cartesian path plan for step 2 (%.2f%% achieved)", fraction_step2 * 100.0);

        if (fraction_step2 < 0.95) {
            RCLCPP_WARN(node->get_logger(), "X-approach path planning failed. Aborting motion.");
        } else {
            RCLCPP_INFO(node->get_logger(), "Path is valid. Executing step 2 (X approach)...");
            move_group_arm.execute(trajectory_step2);

            RCLCPP_INFO(node->get_logger(), "Arrived at target. Pausing for 3 seconds...");
            rclcpp::sleep_for(std::chrono::seconds(3));
        }
        
        // 7. 速度を100%に戻し、初期姿勢へ
        move_group_arm.setMaxVelocityScalingFactor(1.0);
        RCLCPP_INFO(node->get_logger(), "Returning to home position.");
        move_group_arm.setNamedTarget("hr_demo_set");
        move_group_arm.move();
    }
    
    executor->cancel();
    if (spinner_thread.joinable()) {
        spinner_thread.join();
    }
    node.reset();
    rclcpp::shutdown();
    return 0;
}