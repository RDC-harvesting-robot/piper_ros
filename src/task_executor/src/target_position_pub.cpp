#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

class TargetPublisherNode : public rclcpp::Node
{
public:
  TargetPublisherNode() : Node("target_position_pub")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("target_position", 50);
    timer_ = this->create_wall_timer(std::chrono::seconds(2),
                                     std::bind(&TargetPublisherNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Point();
    message.x = 0.25; // 任意の目標値を設定
    message.y = 0.05;
    message.z = 0.2;

    RCLCPP_INFO(this->get_logger(), "Publishing: [%.3f, %.3f, %.3f]", message.x, message.y, message.z);
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
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
