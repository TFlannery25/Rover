// imu_bridge.cpp — reads IMU frames from the STM32 over serial and publishes sensor_msgs/Imu.
// The node polls the port from a timer (no extra thread).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "rover_protocol.h"
#include "serial_port.h"

using namespace std::chrono_literals;

class ImuBridge : public rclcpp::Node {
public:
    ImuBridge() : Node("imu_bridge") {
        port_path_   = declare_parameter<std::string>("serial_port", "/dev/stm32");
        baud_        = declare_parameter<int>("baudrate", 115200);
        frame_id_    = declare_parameter<std::string>("frame_id", "imu_link");
        topic_       = declare_parameter<std::string>("topic", "imu");
        poll_ms_     = declare_parameter<int>("poll_period_ms", 5);
        zero_yaw_    = declare_parameter<bool>("zero_yaw_on_start", true);
        // Variances (sigma^2). First estimates from observed noise at rest; tune later.
        gyro_var_    = declare_parameter<double>("gyro_variance", 0.0025);    // (0.05 rad/s)^2
        accel_var_   = declare_parameter<double>("accel_variance", 0.01);     // (0.1 m/s^2)^2
        yaw_var_     = declare_parameter<double>("yaw_variance", 0.05);       // rad^2, yaw drifts
        // Roll/pitch are not measured by the firmware; large variance = "don't trust".
        tilt_var_    = declare_parameter<double>("roll_pitch_variance", 1000.0);

        pub_ = create_publisher<sensor_msgs::msg::Imu>(topic_, 10);
        poll_timer_  = create_wall_timer(std::chrono::milliseconds(poll_ms_), [this] { poll(); });
        stats_timer_ = create_wall_timer(10s, [this] { logStats(); });

        RCLCPP_INFO(get_logger(), "Publishing /%s (frame %s) from %s @ %d baud",
                    topic_.c_str(), frame_id_.c_str(), port_path_.c_str(), baud_);
        tryOpen();
    }

private:
    void tryOpen() {
        if (port_.open(port_path_, baud_)) {
            RCLCPP_INFO(get_logger(), "Opened %s", port_path_.c_str());
            warned_ = false;
        } else if (!warned_) {
            RCLCPP_WARN(get_logger(), "Cannot open serial port: %s (retrying every second)", port_.lastError().c_str());
            warned_ = true;
        }
    }

    void poll() {
        if (!port_.isOpen()) {
            if (++retry_ticks_ * poll_ms_ >= 1000) { retry_ticks_ = 0; tryOpen(); }
            return;
        }

        batch_.clear();
        uint8_t chunk[256];
        for (;;) {                                           // drain everything currently waiting
            ssize_t n = port_.read(chunk, sizeof chunk);
            if (n < 0) {
                RCLCPP_ERROR(get_logger(), "Serial read failed (%s); closing and retrying", port_.lastError().c_str());
                port_.close();
                parser_ = rover::FrameParser();              // discard any half-received frame
                lastSeq_ = -1;
                break;
            }
            if (n == 0) break;
            parser_.feed(chunk, static_cast<size_t>(n),
                [this](uint8_t type, const uint8_t* payload, size_t len) {
                    if (type == rover::kMsgImu && len == rover::kImuPayloadLen)
                        batch_.push_back(rover::decodeImu(payload));
                });
            if (static_cast<size_t>(n) < sizeof chunk) break;
        }
        if (batch_.empty()) return;

        // Stamp with the Pi's receive time. If several frames arrived in one read, back-date
        // the earlier ones by their STM32 timestamp differences so they stay evenly spaced.
        const rclcpp::Time now = this->now();
        const uint32_t tLast = batch_.back().timestamp_ms;
        for (const auto& m : batch_) {
            const int64_t age_ms = static_cast<int32_t>(tLast - m.timestamp_ms);   // wrap-safe
            publish(m, now - rclcpp::Duration(std::chrono::milliseconds(age_ms)));
        }
    }

    void publish(const rover::ImuMsg& m, const rclcpp::Time& stamp) {
        // sequence bookkeeping (uint8 wraps at 256)
        if (lastSeq_ >= 0) {
            const int diff = (m.seq - lastSeq_) & 0xFF;
            if (diff == 0)      duplicates_++;
            else if (diff != 1) lost_ += diff - 1;
        }
        lastSeq_ = m.seq;
        count_++;

        if (zero_yaw_ && !haveYaw0_) { yaw0_ = m.yaw; haveYaw0_ = true; }
        double yaw = m.yaw - (zero_yaw_ ? yaw0_ : 0.0);
        yaw = std::atan2(std::sin(yaw), std::cos(yaw));      // re-wrap to +-pi

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id_;

        // Firmware sends yaw only, so roll = pitch = 0 here.
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = std::sin(yaw / 2.0);
        msg.orientation.w = std::cos(yaw / 2.0);
        msg.orientation_covariance = {tilt_var_, 0, 0,  0, tilt_var_, 0,  0, 0, yaw_var_};

        msg.angular_velocity.x = m.gyro[0];
        msg.angular_velocity.y = m.gyro[1];
        msg.angular_velocity.z = m.gyro[2];
        msg.angular_velocity_covariance = {gyro_var_, 0, 0,  0, gyro_var_, 0,  0, 0, gyro_var_};

        msg.linear_acceleration.x = m.accel[0];
        msg.linear_acceleration.y = m.accel[1];
        msg.linear_acceleration.z = m.accel[2];
        msg.linear_acceleration_covariance = {accel_var_, 0, 0,  0, accel_var_, 0,  0, 0, accel_var_};

        pub_->publish(msg);
    }

    void logStats() {
        const auto& s = parser_.stats();
        const double hz = static_cast<double>(count_ - countAtLastLog_) / 10.0;
        countAtLastLog_ = count_;
        RCLCPP_INFO(get_logger(), "imu: %.1f Hz  frames=%u crc_fail=%u bad_len=%u lost=%u dup=%u",
                    hz, s.framesOk, s.crcFailures, s.badLengths, lost_, duplicates_);
        if (lost_ > lostAtLastLog_ || s.crcFailures > crcAtLastLog_)
            RCLCPP_WARN(get_logger(), "link errors in the last 10 s: lost=%u crc_fail=%u",
                        lost_ - lostAtLastLog_, s.crcFailures - crcAtLastLog_);
        lostAtLastLog_ = lost_;
        crcAtLastLog_ = s.crcFailures;
    }

    // parameters
    std::string port_path_, frame_id_, topic_;
    int baud_ = 115200, poll_ms_ = 5;
    bool zero_yaw_ = true;
    double gyro_var_, accel_var_, yaw_var_, tilt_var_;

    // layers
    rover::SerialPort port_;
    rover::FrameParser parser_;
    std::vector<rover::ImuMsg> batch_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr poll_timer_, stats_timer_;

    // state
    bool warned_ = false, haveYaw0_ = false;
    int retry_ticks_ = 0, lastSeq_ = -1;
    float yaw0_ = 0.0f;
    uint32_t count_ = 0, countAtLastLog_ = 0, lost_ = 0, lostAtLastLog_ = 0;
    uint32_t duplicates_ = 0, crcAtLastLog_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuBridge>());
    rclcpp::shutdown();
    return 0;
}