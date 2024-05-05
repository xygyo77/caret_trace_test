#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

constexpr int sub_num = 5;

class IntraProcessCommunication : public rclcpp::Node
{
public:
  IntraProcessCommunication() : Node("intra_process_communication", rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", rclcpp::QoS(10).transient_local().durability_volatile());
    
    // 複数のsubscriberを作成する
    for (int i = 0; i < sub_num; ++i) {
        auto subscription_callback = [this, i](const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Subscriber %d received message: '%s'", i, msg->data.c_str());
        };
        subscribers_.push_back(this->create_subscription<std_msgs::msg::String>(
            "topic", rclcpp::QoS(10).transient_local().durability_volatile(), subscription_callback));
    }

    // 一定間隔でメッセージをパブリッシュするタイマーを作成
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        publisher_->publish(message);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscribers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IntraProcessCommunication>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
