#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/create_generic_publisher.hpp"
#include "rclcpp/serialization.hpp"


using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
      publisher_ = this->create_generic_publisher("/ping", "std_msgs/msg/String", rclcpp::QoS(10));
      serializer_ = std::make_shared<rclcpp::Serialization<std_msgs::msg::String>>();
    }

  private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);

        rclcpp::SerializedMessage serialized_message;
        serialized_message.reserve(1024);
        serializer_->serialize_message(&message, &serialized_message);
        publisher_->publish(serialized_message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::GenericPublisher::SharedPtr publisher_;
    std::shared_ptr<rclcpp::Serialization<std_msgs::msg::String>> serializer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}