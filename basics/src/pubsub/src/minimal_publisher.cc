#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rate.hpp>

// Here, I am building a class called "MinimalPublisher". It is a sub-class of
// rclcpp::Node - note that rclcpp stands for R(os)CL(ient)CPP

class MinimalPublisher : public rclcpp::Node
{

private:
  // We will have a "shared pointer" to a rate object:
  rclcpp::Rate::SharedPtr rate_;

  // We will have a "shared pointer" to a publisher:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // We will have a "count" integer.
  int count_;

  // Create the publish new message function:
  void PublishNewMessage()
  {
    // Create a message to send:
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

public:
  // The constructor function for our minimal publisher:
  MinimalPublisher()
      : Node("minimal_publisher"), count_{0}
  {
    // Create our publisher using built-in node-method:
    // HERE, we are publishing a message of type "std_msgs::msg::String".
    // We are publishing over the topic "kTopic"
    // We are keeping the last "10" messages in a queue
    publisher_ = create_publisher<std_msgs::msg::String>("/mae_tutorials/topic", 1);

    // Create our rate - in this case, 1 Hz:
    rate_ = std::make_shared<rclcpp::Rate>(1);
  }

  void RunNode()
  {
    while (rclcpp::ok())
    {
      PublishNewMessage();
      rate_->sleep();
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Note - SPINNING a node is how we start up all of the subscribers, publishers etc.
  // (get it communicating with ROS). But spinning is a BLOCKING call. So we have to call
  // it in a separate thread!
  auto publisher = std::make_shared<MinimalPublisher>();

  // Create an "exectutor" to spin the node in a separate thread:
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  publisher->RunNode();
  rclcpp::shutdown();
  return 0;
}