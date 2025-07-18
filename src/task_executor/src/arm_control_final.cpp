#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <thread>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

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

        move_group_arm.setNamedTarget("hr_demo_set");
        move_group_arm.move();
        
        const auto fixed_orientation = move_group_arm.getCurrentPose().pose.orientation;
        
        RCLCPP_INFO(node->get_logger(), "Reference orientation set. Waiting for target...");

        while (rclcpp::ok() && (!received_target_position || !target_within_threshold)) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        RCLCPP_INFO(node->get_logger(), "Target received. Planning motion...");
        
        const double eef_step = 0.01;
        const double jump_threshold = 5.0;
        const double velocity_scale = 0.1;
        const double acceleration_scale = 1.0;

        // ---(ZY平面での位置合わせ)---
        RCLCPP_INFO(node->get_logger(), "Planning step 1 (ZY alignment) with Cartesian Path...");
        
        std::vector<geometry_msgs::msg::Pose> waypoints_step1;
        geometry_msgs::msg::Pose target_pose_step1 = move_group_arm.getCurrentPose().pose;

        float target_y_offset = latest_target_position.data[0] * -0.001;
        float target_z_offset = latest_target_position.data[1] * 0.001;
        
        target_pose_step1.position.y += target_y_offset + 0.03;
        target_pose_step1.position.z += target_z_offset + 0.08;
        target_pose_step1.orientation = fixed_orientation;
        waypoints_step1.push_back(target_pose_step1);

        moveit_msgs::msg::RobotTrajectory trajectory_step1;
        double fraction_step1 = move_group_arm.computeCartesianPath(waypoints_step1, eef_step, jump_threshold, trajectory_step1);

        RCLCPP_INFO(node->get_logger(), "Cartesian path plan for step 1 (%.2f%% achieved)", fraction_step1 * 100.0);

        if (fraction_step1 < 0.95) {
            RCLCPP_ERROR(node->get_logger(), "Planning failed for step 1. Aborting.");
            rclcpp::shutdown();
            return -1;
        }
        
        robot_trajectory::RobotTrajectory rt_step1(move_group_arm.getRobotModel());
        rt_step1.setRobotTrajectoryMsg(*move_group_arm.getCurrentState(), trajectory_step1);
        
        
        rt_step1.setGroupName(move_group_arm.getName());

        trajectory_processing::IterativeParabolicTimeParameterization time_param_step1;
        bool success_retime_1 = time_param_step1.computeTimeStamps(rt_step1, velocity_scale, acceleration_scale);
        if (!success_retime_1) {
            RCLCPP_ERROR(node->get_logger(), "Failed to re-time trajectory for step 1.");
            return -1;
        }
        rt_step1.getRobotTrajectoryMsg(trajectory_step1);

        RCLCPP_INFO(node->get_logger(), "Executing step 1 (ZY alignment)...");
        move_group_arm.execute(trajectory_step1);


        // --- (X方向へのアプローチ) ---
        RCLCPP_INFO(node->get_logger(), "Planning step 2 (X approach) with Cartesian Path...");
        
        std::vector<geometry_msgs::msg::Pose> waypoints_step2;
        geometry_msgs::msg::Pose start_pose_step2 = move_group_arm.getCurrentPose().pose;
        start_pose_step2.orientation = fixed_orientation;
        waypoints_step2.push_back(start_pose_step2);
        
        float target_x_offset = latest_target_position.data[2] * 0.001;

        geometry_msgs::msg::Pose final_insert_pose = start_pose_step2;
        final_insert_pose.position.x += target_x_offset - 0.20;
        final_insert_pose.orientation = fixed_orientation;
        waypoints_step2.push_back(final_insert_pose);
        
        moveit_msgs::msg::RobotTrajectory trajectory_step2;
        double fraction_step2 = move_group_arm.computeCartesianPath(waypoints_step2, eef_step, jump_threshold, trajectory_step2);

        RCLCPP_INFO(node->get_logger(), "Cartesian path plan for step 2 (%.2f%% achieved)", fraction_step2 * 100.0);

        if (fraction_step2 < 0.95) {
            RCLCPP_WARN(node->get_logger(), "X-approach path planning failed. Aborting motion.");
        } else {
            robot_trajectory::RobotTrajectory rt_step2(move_group_arm.getRobotModel());
            rt_step2.setRobotTrajectoryMsg(*move_group_arm.getCurrentState(), trajectory_step2);

            rt_step2.setGroupName(move_group_arm.getName());

            trajectory_processing::IterativeParabolicTimeParameterization time_param_step2;
            bool success_retime_2 = time_param_step2.computeTimeStamps(rt_step2, velocity_scale, acceleration_scale);
            if (!success_retime_2) {
                RCLCPP_ERROR(node->get_logger(), "Failed to re-time trajectory for step 2.");
                return -1;
            }
            rt_step2.getRobotTrajectoryMsg(trajectory_step2);

            RCLCPP_INFO(node->get_logger(), "Path is valid. Executing step 2 (X approach)...");
            move_group_arm.execute(trajectory_step2);

            RCLCPP_INFO(node->get_logger(), "Arrived at target. Pausing for 3 seconds...");
            rclcpp::sleep_for(std::chrono::seconds(3));
        }
        
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