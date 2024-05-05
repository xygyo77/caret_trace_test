#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

constexpr char NODE_NAME[] = "dummy_node";

class dummyNode : public rclcpp::Node
{
public:
  explicit dummyNode(rclcpp::NodeOptions options)
  : Node(NODE_NAME, options)
  {
    RCLCPP_INFO(this->get_logger(), "Create Node ONLY");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto dum_node = std::make_shared<dummyNode>(rclcpp::NodeOptions());
  rclcpp::spin(dum_node);
  rclcpp::shutdown();
  return 0;
}
