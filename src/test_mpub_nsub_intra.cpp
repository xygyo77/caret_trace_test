#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <vector>
#include <chrono>
#include <memory>

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(const std::string & name, const std::string & topic, int interval)
  : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)), interval_(interval)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS(10));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(interval_),
      [this]() {
        auto message = std_msgs::msg::String();
        message.data = this->get_name() + std::string("Hello, world");
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int interval_;
};

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode(const std::string & name, const std::string & topic)
  : Node(name)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(10).transient_local().durability_volatile(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        //// RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // default
  int num_publishers = 1;
  int num_subscribers_per_publisher = 2;
  int timer = 100;

  // get options
  if (argc > 1) {
    num_publishers = std::stoi(argv[1]);
  }
  if (argc > 2) {
    num_subscribers_per_publisher = std::stoi(argv[2]);
  }
  if (argc > 3) {
    timer = std::stoi(argv[3]);
  }

  // threads
  std::vector<std::thread> threads;

  // (Pub x num_subscribers_per_publisher ; sub x n) x num_publishers 
  for (int i = 0; i < num_publishers; ++i) {
    auto publisher_node = std::make_shared<PublisherNode>("publisher_" + std::to_string(i), "topic_" + std::to_string(i), timer);
    threads.emplace_back([publisher_node]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(publisher_node);
      executor.spin();
      executor.remove_node(publisher_node);
    });

    for (int j = 0; j < num_subscribers_per_publisher; ++j) {
      auto subscriber_node = std::make_shared<SubscriberNode>("subscriber_" + std::to_string(i) + "_" + std::to_string(j), "topic_" + std::to_string(i));
      threads.emplace_back([subscriber_node]() {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(subscriber_node);
        executor.spin();
        executor.remove_node(subscriber_node);
      });
    }
  }

  // threads join
  for (auto & thread : threads) {
    thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
