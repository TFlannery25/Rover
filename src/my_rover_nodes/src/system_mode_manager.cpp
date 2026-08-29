#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_rover_interfaces/srv/set_mode.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <memory>
#include <thread>
#include <termios.h>
#include <unistd.h>

using SetMode = my_rover_interfaces::srv::SetMode;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavClient = rclcpp_action::Client<NavigateToPose>;

enum class Mode { EXPLORE, MANUAL_NAV, FAILSAFE };

std::string mode_to_string(Mode m)
{
  switch (m) {
    case Mode::EXPLORE: return "explore";
    case Mode::MANUAL_NAV: return "manual_nav";
    case Mode::FAILSAFE: return "failsafe";
  }
  return "unknown";
}

class SystemModeManager : public rclcpp::Node
{
public:
  SystemModeManager()
  : Node("system_mode_manager"), mode_(Mode::EXPLORE)
  {
    resume_pub_ = this->create_publisher<std_msgs::msg::Bool>("explore/resume", 10);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("system_mode", 10);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    service_ = this->create_service<SetMode>(
      "set_mode",
      std::bind(&SystemModeManager::handle_set_mode, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "System mode manager ready. Starting in EXPLORE mode.\n"
      "Press 'e' to explore | 'n' for manual nav | 'q' to stop listening\n"
      );

    keyboard_thread_ = std::thread(&SystemModeManager::keyboard_loop, this);
  }

  ~SystemModeManager()
  {
    running_ = false;
    if (keyboard_thread_.joinable()) keyboard_thread_.join();
  }

private:
  char get_key()
  {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
  }

  void keyboard_loop()
  {
    while (running_ && rclcpp::ok()) {
      char key = get_key();
      if (key == 'e') apply_mode(Mode::EXPLORE, false);
      else if (key == 'n') apply_mode(Mode::MANUAL_NAV, false);
      else if (key == 'q') break;
    }
  }

  bool apply_mode(Mode new_mode, bool clear_failsafe)
  {
    if (mode_ == Mode::FAILSAFE && !clear_failsafe) {
      RCLCPP_WARN(this->get_logger(),
        "Ignored mode change: FAILSAFE active. Call /clear_failsafe to resume.");
      return false;
    }

    mode_ = new_mode;

    auto resume_msg = std_msgs::msg::Bool();
    resume_msg.data = (mode_ == Mode::EXPLORE);
    resume_pub_->publish(resume_msg);

    if (mode_ == Mode::FAILSAFE) {
      if (nav_client_->action_server_is_ready()) {
        nav_client_->async_cancel_all_goals();
      }
    }

    RCLCPP_INFO(this->get_logger(), "Mode -> %s", mode_to_string(mode_).c_str());
    auto status_msg = std_msgs::msg::String();
    status_msg.data = mode_to_string(mode_);
    status_pub_->publish(status_msg);
    return true;
  }

  void handle_set_mode(
    const std::shared_ptr<SetMode::Request> request,
    std::shared_ptr<SetMode::Response> response)
  {
    Mode requested;
    if (request->mode == "explore") requested = Mode::EXPLORE;
    else if (request->mode == "manual_nav") requested = Mode::MANUAL_NAV;
    else if (request->mode == "failsafe") requested = Mode::FAILSAFE;
    else {
      response->success = false;
      response->message = "Unknown mode: " + request->mode
        + " (valid: explore, manual_nav, failsafe)";
      return;
    }

    bool applied = apply_mode(requested, request->clear_failsafe);
    response->success = applied;
    response->message = applied
      ? ("Mode set to " + request->mode)
      : "Rejected: failsafe active, clear it first";
  }

  Mode mode_;
  std::atomic<bool> running_{true};
  std::thread keyboard_thread_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resume_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<SetMode>::SharedPtr service_;
  NavClient::SharedPtr nav_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemModeManager>());
  rclcpp::shutdown();
  return 0;
}