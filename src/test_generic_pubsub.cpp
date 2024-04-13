#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

class GenericPublisherSubscriber : public rclcpp::Node
{
public:
    GenericPublisherSubscriber()
    : Node("generic_publisher_subscriber")
    {
        generic_publisher_ = this->create_generic_publisher("/topic", "std_msgs/msg/String", rclcpp::QoS(10));
        //publisher_ = std::dynamic_pointer_cast<rclcpp::PublisherBase>(generic_publisher);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/topic", 10, std::bind(&GenericPublisherSubscriber::topic_callback, this, std::placeholders::_1));
    }

    void publish_message(const std_msgs::msg::String &msg)
    {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<std_msgs::msg::String> serialization;
        serialization.serialize_message(&msg, &serialized_msg);

        //auto generic_publisher = std::dynamic_pointer_cast<rclcpp::GenericPublisher>(publisher_);
        if (generic_publisher_) {
            generic_publisher_->publish(serialized_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Publisher cast to GenericPublisher failed.");
        }
}

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    rclcpp::GenericPublisher::SharedPtr generic_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenericPublisherSubscriber>();

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
