#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class CropTFBroadcaster : public rclcpp::Node
{
public:
  CropTFBroadcaster() : Node("crop_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/crop_cordinate", 10,
      std::bind(&CropTFBroadcaster::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3)
    {
      RCLCPP_WARN(this->get_logger(), "Received data too small.");
      return;
    }

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "camera_link";  // カメラのTF名に合わせて
    transformStamped.child_frame_id = "target_object";

    transformStamped.transform.translation.x = msg->data[2] * 0.001;
    transformStamped.transform.translation.y = msg->data[1] * -0.001;
    transformStamped.transform.translation.z = msg->data[0] * -0.001;

    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transformStamped);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CropTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
