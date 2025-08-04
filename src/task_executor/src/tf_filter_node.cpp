#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>
#include <string>

class TfFilterNode : public rclcpp::Node
{
public:
  TfFilterNode()
  : Node("tf_filter_node")
  {
    // 表示したいリンク名
    target_frames_ = {"target_object"}; 

    sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10,
      std::bind(&TfFilterNode::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_filtered", 10);
  }

private:
  void callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    tf2_msgs::msg::TFMessage filtered_msg;
    for (const auto& transform : msg->transforms)
    {
      for (const auto& target : target_frames_)
      {
        if (transform.child_frame_id == target)
        {
          filtered_msg.transforms.push_back(transform);
          break;
        }
      }
    }

    if (!filtered_msg.transforms.empty())
    {
      pub_->publish(filtered_msg);
    }
  }

  std::vector<std::string> target_frames_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfFilterNode>());
  rclcpp::shutdown();
  return 0;
}
