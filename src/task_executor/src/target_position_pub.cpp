#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

class TargetPublisherNode : public rclcpp::Node
{
public:
  TargetPublisherNode() : Node("target_position_pub")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("target_position", 500);
    timer_ = this->create_wall_timer(std::chrono::seconds(2),
                                     std::bind(&TargetPublisherNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32MultiArray();

    // 任意の目標値を設定
    message.data = {200, 0, 0};

    RCLCPP_INFO(this->get_logger(), "Publishing: [%d, %d, %d]", 
                message.data[0], message.data[1], message.data[2]);
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TargetPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}