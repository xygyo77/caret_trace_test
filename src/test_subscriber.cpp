#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

constexpr char NODE_NAME[] = "subscriber_node";
constexpr char TOPIC_NAME[] = "/chatter";

class SubscriberNode : public rclcpp::Node
{
public:
  explicit SubscriberNode(rclcpp::NodeOptions options)
  : Node(NODE_NAME, options)
  {
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      TOPIC_NAME,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto subscriber_node = std::make_shared<SubscriberNode>(rclcpp::NodeOptions());
  rclcpp::spin(subscriber_node);
  rclcpp::shutdown();
  return 0;
}
