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
  : Node(name), interval_(interval)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS(10));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(interval_),
      [this]() {
        auto message = std_msgs::msg::String();
        message.data = std::string(this->get_name()) + " Hello, world!";
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
      topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int num_publishers = 300;
  int num_subscribers_per_publisher = 5;
  int timer = 100;

  if (argc > 1) {
    num_publishers = std::stoi(argv[1]);
  }
  if (argc > 2) {
    num_subscribers_per_publisher = std::stoi(argv[2]);
  }
  if (argc > 3) {
    timer = std::stoi(argv[3]);
  }

  std::vector<std::shared_ptr<rclcpp::Node>> nodes;
  std::vector<std::thread> threads;
  rclcpp::executors::MultiThreadedExecutor executor;

  try {
    for (int i = 0; i < num_publishers; ++i) {
      auto publisher_node = std::make_shared<PublisherNode>("pub_" + std::to_string(i), "t_" + std::to_string(i), timer);
      nodes.push_back(publisher_node);

      for (int j = 0; j < num_subscribers_per_publisher; ++j) {
        auto subscriber_node = std::make_shared<SubscriberNode>("sub_" + std::to_string(i) + "_" + std::to_string(j), "t_" + std::to_string(i));
        nodes.push_back(subscriber_node);
      }
    }

    for (auto & node : nodes) {
      executor.add_node(node);
    }

    threads.emplace_back([&executor]() {
      executor.spin();
    });

    for (auto & thread : threads) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  } catch (const rclcpp::exceptions::RCLError & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Caught exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
