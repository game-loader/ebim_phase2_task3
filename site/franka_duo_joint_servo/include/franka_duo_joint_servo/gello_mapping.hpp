#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace franka_duo_joint_servo {

// Hamburg controller: q_goal = robot_ref + D * (gello - gello_ref).
// Hold gello == measured at activation, then invert D (D * D == I).
class GelloMapping {
 public:
  using Joints = std::array<double, 7>;
  inline static constexpr Joints direction{-1, -1, 1, 1, 1, 1, -1};

  void observe(const Joints& measured) {
    validate(measured);
    measured_ = measured;
    observed_ = true;
  }
  void prepare() {
    if (!observed_ || prepared_) {
      throw std::logic_error("reference unavailable or already latched; restart inactive runtime to rebase");
    }
    reference_ = measured_;
    prepared_ = true;
  }
  void enable() {
    if (!prepared_ || enabled_) {
      throw std::logic_error("mapping must be prepared exactly once before enabling");
    }
    enabled_ = true;
  }
  Joints encode(const Joints& target) const {
    validate(target);
    if (!observed_) {
      throw std::logic_error("measured joints unavailable");
    }
    if (!prepared_) return measured_;
    if (!enabled_) return reference_;
    Joints output{};
    for (size_t i = 0; i < output.size(); ++i) {
      output[i] = reference_[i] + direction[i] * (target[i] - reference_[i]);
    }
    return output;
  }
  bool observed() const { return observed_; }
  bool prepared() const { return prepared_; }
  bool enabled() const { return enabled_; }
  const Joints& reference() const { return reference_; }

 private:
  static void validate(const Joints& values) {
    for (double v : values) {
      if (!std::isfinite(v)) throw std::invalid_argument("non-finite joint position");
    }
  }
  Joints measured_{}, reference_{};
  bool observed_{false}, prepared_{false}, enabled_{false};
};
}  // namespace franka_duo_joint_servo
