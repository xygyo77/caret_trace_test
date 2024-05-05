#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

constexpr char NODE_NAME[] = "publisher_node";
constexpr char TOPIC_NAME[] = "/chatter";

class PublisherNode : public rclcpp::Node
{
public:
  explicit PublisherNode(rclcpp::NodeOptions options)
  : Node(NODE_NAME, options)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      TOPIC_NAME,
      rclcpp::QoS(10));
  }

  void start_publishing()
  {
    // Publish message periodically
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, INTER world!";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publish: '%s'", message.data.c_str());
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto publisher_node = std::make_shared<PublisherNode>(rclcpp::NodeOptions());
  publisher_node->start_publishing(); // タイマを開始する
  rclcpp::spin(publisher_node);
  rclcpp::shutdown();
  return 0;
}
