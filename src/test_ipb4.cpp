#include "rclcpp/rclcpp.hpp"
#include "rclcpp/experimental/buffers/intra_process_buffer.hpp"
#include "std_msgs/msg/string.hpp"

class IntraProcessCommunication : public rclcpp::Node
{
public:
  IntraProcessCommunication() : Node("intra_process_communication")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", rclcpp::QoS(10).transient_local());
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "topic", rclcpp::QoS(10).transient_local(),
      std::bind(&IntraProcessCommunication::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IntraProcessCommunication>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
