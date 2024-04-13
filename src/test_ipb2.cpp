#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class IntraProcessPublisher : public rclcpp::Node
{
public:
    IntraProcessPublisher() : Node("intra_process_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(1s, std::bind(&IntraProcessPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

class IntraProcessSubscriber : public rclcpp::Node
{
public:
    IntraProcessSubscriber() : Node("intra_process_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&IntraProcessSubscriber::callback, this, std::placeholders::_1));
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr message)
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", message->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto publisher_node = std::make_shared<IntraProcessPublisher>();
    auto subscriber_node = std::make_shared<IntraProcessSubscriber>();

    rclcpp::spin(publisher_node);
    rclcpp::spin(subscriber_node);

    rclcpp::shutdown();
    return 0;
}
