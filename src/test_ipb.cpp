#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher() : Node("my_publisher")
  {
    // Use IntraProcessBuffer to enable intra-process communication
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic", rclcpp::QoS(10));
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MyPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = 42;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.data);
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class MySubscriber : public rclcpp::Node
{
public:
  MySubscriber() : Node("my_subscriber")
  {
    // Use IntraProcessBuffer to enable intra-process communication
    subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
  "topic", rclcpp::QoS(10),
  std::bind(&MySubscriber::topic_callback, this, std::placeholders::_1));

  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  auto publisher_node = std::make_shared<MyPublisher>();
  auto subscriber_node = std::make_shared<MySubscriber>();

  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}