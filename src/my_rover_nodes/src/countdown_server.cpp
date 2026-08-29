#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_rover_interfaces/action/countdown.hpp"

#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using Countdown = my_rover_interfaces::action::Countdown;
using GoalHandleCountdown = rclcpp_action::ServerGoalHandle<Countdown>;

class CountdownServer : public rclcpp::Node
{
public:
  CountdownServer()
  : Node("countdown_server")
  {
    action_server_ = rclcpp_action::create_server<Countdown>(
      this,
      "countdown",
      std::bind(&CountdownServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CountdownServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&CountdownServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Countdown action server ready.");
  }

private:
  rclcpp_action::Server<Countdown>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Countdown::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: count down from %d", goal->starting_number);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCountdown>)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCountdown> goal_handle)
  {
    std::thread{std::bind(&CountdownServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCountdown> goal_handle)
  {
    int start = goal_handle->get_goal()->starting_number;
    auto feedback = std::make_shared<Countdown::Feedback>();
    auto result = std::make_shared<Countdown::Result>();

    for (int i = start; i >= 0; --i) {
      if (goal_handle->is_canceling()) {
        result->final_number = i;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Countdown canceled at %d", i);
        return;
      }

      feedback->current_count = i;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Countdown: %d", i);

      std::this_thread::sleep_for(1s);
    }

    result->final_number = 0;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Countdown complete!");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CountdownServer>());
  rclcpp::shutdown();
  return 0;
}