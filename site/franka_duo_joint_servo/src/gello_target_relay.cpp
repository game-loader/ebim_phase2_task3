// Site-owned gate between the joint servo and the site JointImpedanceController.
//
// Forwards sensor_msgs/JointState targets to the controller input topic
// (relative "gello/joint_states" inside the arm namespace) only when
// enable_robot is true, and Float32 gripper targets to the site gripper
// client only when enable_gripper is true.  Targets must carry the expected
// seven joint names in order, finite values and a fresh stamp.
// Hamburg gello_relative mode holds measured joints across the explicit
// activation handshake, then maps robot-space targets into GELLO coordinates.

#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "franka_duo_joint_servo/gello_mapping.hpp"

class GelloTargetRelay final : public rclcpp::Node {
 public:
  GelloTargetRelay() : Node("gello_target_relay") {
    declare_parameter<std::string>("input_topic", "");
    declare_parameter<std::string>("output_topic", "");
    declare_parameter<std::string>("gripper_input_topic", "");
    declare_parameter<std::string>("gripper_output_topic", "");
    declare_parameter<std::vector<std::string>>("expected_joint_names", {});
    declare_parameter<double>("max_target_age_s", 0.1);
    declare_parameter<bool>("enable_robot", false);
    declare_parameter<bool>("enable_gripper", false);
    // The site JointImpedanceController shuts the whole driver down when the
    // next target arrives more than 0.5 s after the previous one.  When the
    // servo stops publishing, keep re-sending the last forwarded target with
    // a fresh stamp so the arm holds its pose instead of losing the driver.
    declare_parameter<bool>("hold_on_input_loss", true);
    declare_parameter<double>("output_rate_hz", 0.0);
    declare_parameter<std::string>("command_mode", "absolute");
    declare_parameter<std::string>("measured_topic", "");

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    gripper_input_topic_ = get_parameter("gripper_input_topic").as_string();
    gripper_output_topic_ = get_parameter("gripper_output_topic").as_string();
    expected_joint_names_ = get_parameter("expected_joint_names").as_string_array();
    max_target_age_s_ = get_parameter("max_target_age_s").as_double();
    enable_robot_ = get_parameter("enable_robot").as_bool();
    enable_gripper_ = get_parameter("enable_gripper").as_bool();
    hold_on_input_loss_ = get_parameter("hold_on_input_loss").as_bool();
    output_rate_hz_ = get_parameter("output_rate_hz").as_double();
    const auto mode = get_parameter("command_mode").as_string();
    if (mode != "absolute" && mode != "gello_relative") {
      throw std::invalid_argument("unknown command_mode");
    }
    relative_ = mode == "gello_relative";
    if (!std::isfinite(output_rate_hz_) || output_rate_hz_ < 0.0 ||
        (output_rate_hz_ > 0.0 && output_rate_hz_ < 10.0)) {
      throw std::invalid_argument("output_rate_hz must be zero (passthrough) or at least 10 Hz");
    }
    if (input_topic_.empty() || output_topic_.empty() || expected_joint_names_.size() != 7U) {
      throw std::invalid_argument(
          "input_topic, output_topic and seven expected_joint_names are required");
    }
    if (input_topic_ == output_topic_) {
      throw std::invalid_argument("input_topic and output_topic must differ");
    }
    if (!std::isfinite(max_target_age_s_) || max_target_age_s_ <= 0.0 || max_target_age_s_ >= 0.5) {
      throw std::invalid_argument("max_target_age_s must be in (0, 0.5)");
    }

    publisher_ = create_publisher<sensor_msgs::msg::JointState>(output_topic_, rclcpp::QoS(1).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        input_topic_, rclcpp::QoS(1).reliable(),
        [this](const sensor_msgs::msg::JointState::SharedPtr message) { forward(*message); });
    if (relative_) {
      const auto topic = get_parameter("measured_topic").as_string();
      if (topic.empty() || output_rate_hz_ <= 0.0 || !hold_on_input_loss_) {
        throw std::invalid_argument("gello_relative requires measured_topic, periodic output and hold_on_input_loss");
      }
      measured_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
          topic, rclcpp::SensorDataQoS(),
          [this](const sensor_msgs::msg::JointState::SharedPtr message) { measured(*message); });
      mapping_status_ = create_publisher<std_msgs::msg::String>("~/mapping_status", 10);
      prepare_service_ = create_service<std_srvs::srv::Trigger>("~/prepare_mapping",
          [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response) { transition(false, *response); });
      enable_service_ = create_service<std_srvs::srv::Trigger>("~/enable_mapping",
          [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response) { transition(true, *response); });
    }
    if (!gripper_input_topic_.empty() && !gripper_output_topic_.empty()) {
      gripper_publisher_ =
          create_publisher<std_msgs::msg::Float32>(gripper_output_topic_, rclcpp::QoS(10).reliable());
      gripper_subscription_ = create_subscription<std_msgs::msg::Float32>(
          gripper_input_topic_, rclcpp::QoS(10).reliable(),
          [this](const std_msgs::msg::Float32::SharedPtr message) { forwardGripper(*message); });
    }
    RCLCPP_INFO(
        get_logger(), "Relay %s -> %s; mode=%s enable_robot=%s enable_gripper=%s", input_topic_.c_str(),
        output_topic_.c_str(), mode.c_str(), enable_robot_ ? "true" : "false", enable_gripper_ ? "true" : "false");
    if (enable_robot_) {
      RCLCPP_WARN(get_logger(), "Robot output ENABLED: targets reach the joint impedance controller");
    }
    if (output_rate_hz_ > 0) {
      hold_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / output_rate_hz_), [this] {
        if (enable_robot_ && have_last_) {
          auto held = last_forwarded_;
          held.header.stamp = now();
          publisher_->publish(held);
        }
        if (relative_) publishMappingStatus();
      });
    } else if (hold_on_input_loss_) {
      hold_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { holdTick(); });
    }
  }

 private:
  using Mapping = franka_duo_joint_servo::GelloMapping;

  bool freshStamp(const sensor_msgs::msg::JointState& message) const {
    const double age = (now() - rclcpp::Time(message.header.stamp)).seconds();
    return age >= -max_target_age_s_ && age <= max_target_age_s_;
  }

  void measured(const sensor_msgs::msg::JointState& message) {
    if (!freshStamp(message) || message.name.size() != message.position.size() ||
        std::set<std::string>(message.name.begin(), message.name.end()).size() != message.name.size()) return;
    Mapping::Joints ordered{};
    for (size_t i = 0; i < ordered.size(); ++i) {
      auto it = std::find(message.name.begin(), message.name.end(), expected_joint_names_[i]);
      if (it == message.name.end()) return;
      ordered[i] = message.position[std::distance(message.name.begin(), it)];
      if (!std::isfinite(ordered[i])) return;
    }
    mapping_.observe(ordered);
    last_measured_ = message;
    // No servo target, however distant, may become the activation reference.
    if (!mapping_.prepared()) storeMapped(ordered);
  }

  void storeMapped(const Mapping::Joints& target) {
    const auto encoded = mapping_.encode(target);
    last_forwarded_.name = expected_joint_names_;
    last_forwarded_.position.assign(encoded.begin(), encoded.end());
    // Only positions are consumed by this interface. Do not leak robot-space
    // velocity/effort arrays into GELLO coordinates.
    last_forwarded_.velocity.clear();
    last_forwarded_.effort.clear();
    last_forwarded_.header.stamp = now();
    last_forward_time_ = std::chrono::steady_clock::now();
    have_last_ = true;
  }

  void transition(bool enable, std_srvs::srv::Trigger::Response& response) {
    try {
      if (!enable_robot_ || !mapping_.observed() || !freshStamp(last_measured_) ||
          !valid(last_target_)) throw std::logic_error("fresh measured joints and servo target required");
      if (enable) mapping_.enable(); else mapping_.prepare();
      Mapping::Joints target{};
      std::copy_n(last_target_.position.begin(), 7, target.begin());
      storeMapped(target);
      response.success = true;
    } catch (const std::exception& error) {
      response.success = false;
      response.message = error.what();
    }
    publishMappingStatus();
  }

  void publishMappingStatus() {
    std::ostringstream stream;
    stream.precision(17);
    stream << "{\"state\":\"" << (mapping_.enabled() ? "enabled" :
        mapping_.prepared() ? "reference_held" : "following_measured") << "\",\"reference\":[";
    for (size_t i = 0; i < 7; ++i) stream << (i ? "," : "") << mapping_.reference()[i];
    stream << "]}";
    std_msgs::msg::String message;
    message.data = stream.str();
    mapping_status_->publish(message);
  }

  void holdTick() {
    if (!enable_robot_ || !have_last_) {
      return;
    }
    const double age = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - last_forward_time_).count();
    if (age < 0.02) {
      return;
    }
    sensor_msgs::msg::JointState held = last_forwarded_;
    held.header.stamp = now();
    publisher_->publish(held);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No target from %s for %.2f s; holding the last forwarded joint target", input_topic_.c_str(), age);
  }

  bool valid(const sensor_msgs::msg::JointState& message) const {
    if (message.name != expected_joint_names_ || message.position.size() != 7U) {
      return false;
    }
    for (const auto value : message.position) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    const double age = (now() - rclcpp::Time(message.header.stamp)).seconds();
    return age >= -max_target_age_s_ && age <= max_target_age_s_;
  }

  void forward(const sensor_msgs::msg::JointState& message) {
    if (!valid(message)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Dropped invalid or stale joint target from %s",
          input_topic_.c_str());
      return;
    }
    if (!enable_robot_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Robot output disabled; dropping targets from %s",
          input_topic_.c_str());
      return;
    }
    if (relative_) {
      last_target_ = message;
      if (mapping_.observed()) {
        Mapping::Joints target{};
        std::copy_n(message.position.begin(), 7, target.begin());
        storeMapped(target);
      }
      return;
    }
    if (output_rate_hz_ == 0.0) {
      publisher_->publish(message);
    }
    last_forwarded_ = message;
    last_forward_time_ = std::chrono::steady_clock::now();
    have_last_ = true;
  }

  void forwardGripper(const std_msgs::msg::Float32& message) {
    if (!std::isfinite(message.data) || message.data < 0.0F || message.data > 1.0F) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropped gripper target outside [0, 1]");
      return;
    }
    if (!enable_gripper_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Gripper output disabled; dropping target");
      return;
    }
    gripper_publisher_->publish(message);
  }

  std::string input_topic_, output_topic_, gripper_input_topic_, gripper_output_topic_;
  std::vector<std::string> expected_joint_names_;
  double max_target_age_s_{0.1};
  bool enable_robot_{false};
  bool enable_gripper_{false};
  bool hold_on_input_loss_{true};
  double output_rate_hz_{0.0};
  bool have_last_{false};
  bool relative_{false};
  Mapping mapping_;
  sensor_msgs::msg::JointState last_measured_, last_target_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mapping_status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_service_, enable_service_;
  sensor_msgs::msg::JointState last_forwarded_;
  std::chrono::steady_clock::time_point last_forward_time_{};
  rclcpp::TimerBase::SharedPtr hold_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GelloTargetRelay>());
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("gello_target_relay"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
