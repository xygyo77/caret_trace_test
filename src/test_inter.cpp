// Copyright 2019 Robert Bosch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#define PUB_NODE_NAME "test_inter_pub"
#define SUB_NODE_NAME "test_inter_sub"
#define TOPIC_NAME "the_topic"

class PubInterNode : public rclcpp::Node
{
public:
  explicit PubInterNode(const std::string & name)
  : Node(name)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>(
      TOPIC_NAME,
      rclcpp::QoS(10));
    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&PubInterNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "some random interprocess string";
    pub_->publish(*msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class SubInterNode : public rclcpp::Node
{
public:
  explicit SubInterNode(const std::string & name)
  : Node(name)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      TOPIC_NAME,
      rclcpp::QoS(10),
      std::bind(&SubInterNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[output] %s", msg->data.c_str());
    rclcpp::shutdown();
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto pub_inter_node = std::make_shared<PubInterNode>(PUB_NODE_NAME);
  auto sub_inter_node = std::make_shared<SubInterNode>(SUB_NODE_NAME);
  exec.add_node(pub_inter_node);
  exec.add_node(sub_inter_node);

  printf("spinning\n");
  exec.spin();

  // Will actually be called inside the node's callback
  rclcpp::shutdown();
  return 0;
}
