#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "my_rover_interfaces/srv/set_mode.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavClient = rclcpp_action::Client<NavigateToPose>;
using SetMode = my_rover_interfaces::srv::SetMode;

class SafetyMonitor : public rclcpp::Node
{
public:
  SafetyMonitor()
  : Node("safety_monitor"), triggered_(false)
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&SafetyMonitor::scan_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10,
      std::bind(&SafetyMonitor::odom_callback, this, std::placeholders::_1));

    status_pub_ = this->create_publisher<std_msgs::msg::String>("safety_status", 10);

    clear_service_ = this->create_service<std_srvs::srv::Trigger>(
      "clear_failsafe",
      std::bind(&SafetyMonitor::handle_clear, this,
        std::placeholders::_1, std::placeholders::_2));

    mode_client_ = this->create_client<SetMode>("set_mode");
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    last_odom_time_ = this->now();

    // Watchdog: checks every second whether odometry has gone stale
    watchdog_timer_ = this->create_wall_timer(
      1s, std::bind(&SafetyMonitor::check_watchdog, this));

    RCLCPP_INFO(this->get_logger(), "Safety monitor active.");
  }

private:
  static constexpr double MIN_SAFE_DISTANCE = 0.25;  // meters
  static constexpr double ODOM_TIMEOUT_SEC = 2.0;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    for (float range : msg->ranges) {
      if (range > 0.0 && range < MIN_SAFE_DISTANCE) {
        trigger_failsafe("Obstacle too close: " + std::to_string(range) + "m");
        return;
      }
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr)
  {
    last_odom_time_ = this->now();
  }

  void check_watchdog()
  {
    double elapsed = (this->now() - last_odom_time_).seconds();
    if (elapsed > ODOM_TIMEOUT_SEC && !triggered_) {
      trigger_failsafe("Odometry data stale for " + std::to_string(elapsed) + "s");
    }
  }

  void trigger_failsafe(const std::string & reason)
  {
    if (triggered_) {
      return;  // already stopped, don't spam
    }
    triggered_ = true;

    RCLCPP_WARN(this->get_logger(), "FAILSAFE TRIGGERED: %s", reason.c_str());
    publish_status("TRIGGERED: " + reason);

    if (nav_client_->action_server_is_ready()) {
      nav_client_->async_cancel_all_goals();
    }

    if (mode_client_->service_is_ready()) {
      auto request = std::make_shared<SetMode::Request>();
      request->mode = "failsafe";
      request->clear_failsafe = false;
      mode_client_->async_send_request(request);
    } else {
      RCLCPP_WARN(this->get_logger(),
        "system_mode_manager service not available; failsafe applied locally only.");
    }
  }

  void handle_clear(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    triggered_ = false;
    last_odom_time_ = this->now();  // reset watchdog clock on clear

    if (mode_client_->service_is_ready()) {
      auto request = std::make_shared<SetMode::Request>();
      request->mode = "manual_nav";  // safe default to return to
      request->clear_failsafe = true;
      mode_client_->async_send_request(request);
    }

    RCLCPP_INFO(this->get_logger(), "Failsafe cleared. Resuming normal operation.");
    publish_status("OK");
    response->success = true;
    response->message = "Failsafe cleared";
  }

  void publish_status(const std::string & status)
  {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_pub_->publish(msg);
  }

  bool triggered_;
  rclcpp::Time last_odom_time_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  rclcpp::Client<SetMode>::SharedPtr mode_client_;
  NavClient::SharedPtr nav_client_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}