#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_rover_interfaces/action/countdown.hpp"

#include <memory>

using Countdown = my_rover_interfaces::action::Countdown;
using GoalHandleCountdown = rclcpp_action::ClientGoalHandle<Countdown>;

class CountdownClient : public rclcpp::Node
{
public:
  CountdownClient()
  : Node("countdown_client")
  {
    client_ = rclcpp_action::create_client<Countdown>(this, "countdown");
  }

  void send_goal(int starting_number)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available.");
      return;
    }

    auto goal_msg = Countdown::Goal();
    goal_msg.starting_number = starting_number;

    auto send_goal_options = rclcpp_action::Client<Countdown>::SendGoalOptions();

    send_goal_options.feedback_callback =
      [this](GoalHandleCountdown::SharedPtr, const std::shared_ptr<const Countdown::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Feedback: %d", feedback->current_count);
      };

    send_goal_options.result_callback =
      [this](const GoalHandleCountdown::WrappedResult & result) {
        RCLCPP_INFO(this->get_logger(), "Result: reached %d", result.result->final_number);
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Countdown>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CountdownClient>();
  node->send_goal(5);
  rclcpp::spin(node);
  return 0;
}