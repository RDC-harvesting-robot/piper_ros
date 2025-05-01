#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/pose_set.hpp"  // ここは正しいsrvパスに変更してください

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pose_set_client");

  auto client = node->create_client<example_interfaces::srv::PoseSet>("/pose_set");

  // サーバーが起動するまで待つ
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "サービスが見つかりません");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Waiting for target position...");
  auto request = std::make_shared<example_interfaces::srv::PoseSet::Request>();
  RCLCPP_INFO(node->get_logger(), ">>>>>>>>>>>>>>>>>>");
  request->action = "close";
  RCLCPP_INFO(node->get_logger(), "???????????????");
  auto result_future = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  rclcpp::sleep_for(std::chrono::milliseconds(5000));

  RCLCPP_INFO(node->get_logger(), "Waiting for target position...");
  auto request_2 = std::make_shared<example_interfaces::srv::PoseSet::Request>();
  request_2->action = "open";
  auto result_future_2 = client->async_send_request(request_2);


  // 応答を待機
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    RCLCPP_INFO(node->get_logger(), "Response: success = %s", response->success ? "true" : "false");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "サービス呼び出しに失敗しました");
  }

  rclcpp::shutdown();
  return 0;
}
