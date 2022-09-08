#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <functional>

#include <memory>

class HeavyMathCppNode : public rclcpp::Node
{
private:
  // The service:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr adder_;

  void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    // Perform the math that is too heavy to be possible in python:
    response->sum = request->a + request->b;

    // log for ourselves:
    RCLCPP_INFO_STREAM(get_logger(), "sending back response: " << response->sum);
  }

public:
  HeavyMathCppNode()
      : rclcpp::Node("heavy_math_node")
  {
    // Initialize the service:
    adder_ = create_service<example_interfaces::srv::AddTwoInts>(                               // the template is the "srv" type
        "/mae_tutorials/add_two_ints",                                                          // the topic
        std::bind(&HeavyMathCppNode::add, this, std::placeholders::_1, std::placeholders::_2)); // the callback

    RCLCPP_INFO_STREAM(get_logger(), "Ready to perform heavy math!");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto heavy_math_node = std::make_shared<HeavyMathCppNode>();

  // blocking call - but that is okay, since we aren't doing anything else.
  rclcpp::spin(heavy_math_node);
  rclcpp::shutdown();
}