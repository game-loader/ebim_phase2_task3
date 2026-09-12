// Offline FK/IK round trips. This executable never creates command publishers.
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>(
        "duo_model_check", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    robot_model_loader::RobotModelLoader loader(node);
    const auto model = loader.getModel();
    if (!model) {
      throw std::runtime_error("robot model unavailable");
    }
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    for (const std::string side : {"left", "right"}) {
      const auto* group = model->getJointModelGroup(side + "_arm");
      if (!group || group->getVariableCount() != 7 || !group->getSolverInstance()) {
        throw std::runtime_error(side + " seven-joint KDL solver unavailable");
      }
      const std::string tip = side + "_fr3v2_link8";
      for (int sample = 0; sample < 3; ++sample) {
        const std::vector<double> joints{0.1 * sample, -0.7, 0.1, -2.0, 0.0, 1.5, 0.1};
        state.setJointGroupPositions(group, joints);
        state.update();
        const Eigen::Isometry3d target = state.getGlobalLinkTransform(tip);
        auto seed = joints;
        seed[0] += 0.03;
        state.setJointGroupPositions(group, seed);
        if (!state.setFromIK(group, target, tip, 0.1)) {
          throw std::runtime_error(side + " IK round trip failed");
        }
        state.update();
        const Eigen::Isometry3d actual = state.getGlobalLinkTransform(tip);
        const double position_error = (actual.translation() - target.translation()).norm();
        const double rotation_error = Eigen::AngleAxisd(target.rotation().transpose() * actual.rotation()).angle();
        if (!std::isfinite(position_error) || !std::isfinite(rotation_error) ||
            position_error > 1e-4 || rotation_error > 1e-3 || !state.satisfiesBounds(group)) {
          throw std::runtime_error(side + " FK/IK mismatch or joint limit violation");
        }
        std::cout << side << " sample=" << sample << " position_error_m=" << position_error
                  << " rotation_error_rad=" << rotation_error << '\n';
      }
    }
    std::cout << "MODEL_CHECK_PASSED: both seven-joint KDL solvers; no robot commands sent\n";
  } catch (const std::exception& error) {
    std::cerr << "MODEL_CHECK_FAILED: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
