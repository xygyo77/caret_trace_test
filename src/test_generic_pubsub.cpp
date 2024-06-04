#include <memory>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"

class GenericPublisherSubscriber : public rclcpp::Node
{
public:
  GenericPublisherSubscriber(const std::string & node_name)
  : Node(node_name)
  {
    generic_publisher_ = this->create_generic_publisher(
      "/topic",
      "std_msgs/msg/String",
      rclcpp::QoS(10));

    auto callback = [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      RCLCPP_INFO(this->get_logger(), "Received message");
    };

    generic_subscription_ = this->create_generic_subscription(
      "/topic",
      "std_msgs/msg/String",
      rclcpp::QoS(10),
      callback);
  }

  void publish_message(const std_msgs::msg::String &msg)
  {
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<std_msgs::msg::String> serialization;
      serialization.serialize_message(&msg, &serialized_msg);

      if (generic_publisher_) {
          generic_publisher_->publish(serialized_msg);
      } else {
          RCLCPP_ERROR(this->get_logger(), "Publisher cast to GenericPublisher failed.");
      }
  }

private:
  rclcpp::GenericPublisher::SharedPtr generic_publisher_;
  rclcpp::GenericSubscription::SharedPtr generic_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GenericPublisherSubscriber>("GenericPublisherSubscriber");

  for (int i = 0; i < 5; i++) {
    std_msgs::msg::String message;
    message.data = "Hello, world!";
    node->publish_message(message);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
