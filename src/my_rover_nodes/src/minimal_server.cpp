#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

void handle_service(
  const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
  std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("minimal_server"),
    "Incoming request: %ld + %ld", request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("minimal_server"),
    "Sending back response: %ld", response->sum);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("minimal_server");

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints", &handle_service);

  RCLCPP_INFO(rclcpp::get_logger("minimal_server"), "Ready to add two ints.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}