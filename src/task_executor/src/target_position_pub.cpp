#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

class PosePublisherNode : public rclcpp::Node
{
public:
  PosePublisherNode() : Node("target_position_pub")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PosePublisherNode::publish_pose_and_tf, this));
  }

private:
  void publish_pose_and_tf()
  {
    std::string yaml_path = "/home/yugonishio/ros2_ws/src/piper_ros/src/task_executor/config/last_tf_pose.yaml";
    YAML::Node node;
    try
    {
      node = YAML::LoadFile(yaml_path);
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to load YAML file: %s", e.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->get_clock()->now();
    pose.header.frame_id = "base_link";  // TFの親フレーム
    pose.pose.position.x = node["position"]["x"].as<double>();
    pose.pose.position.y = node["position"]["y"].as<double>();
    pose.pose.position.z = node["position"]["z"].as<double>();
    pose.pose.orientation.x = node["orientation"]["x"].as<double>();
    pose.pose.orientation.y = node["orientation"]["y"].as<double>();
    pose.pose.orientation.z = node["orientation"]["z"].as<double>();
    pose.pose.orientation.w = node["orientation"]["w"].as<double>();

    pub_->publish(pose);

    // TFブロードキャスト
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = pose.header.stamp;
    tf_msg.header.frame_id = "base_link";
    tf_msg.child_frame_id = "target_object";
    tf_msg.transform.translation.x = pose.pose.position.x;
    tf_msg.transform.translation.y = pose.pose.position.y;
    tf_msg.transform.translation.z = pose.pose.position.z;
    tf_msg.transform.rotation = pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosePublisherNode>());
  rclcpp::shutdown();
  return 0;
}
