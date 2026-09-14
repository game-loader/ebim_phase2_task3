#include <gtest/gtest.h>
#include <limits>
#include "franka_duo_joint_servo/gello_mapping.hpp"

using Mapping = franka_duo_joint_servo::GelloMapping;

TEST(GelloMapping, HoldsMeasuredThenFreezesReferenceBeforeEnable) {
  Mapping map;
  Mapping::Joints distant{1, 2, 3, 4, 5, 6, 7};
  Mapping::Joints measured{0.2, -0.7, 0.1, -2, 0.3, 1.5, -0.1};
  EXPECT_THROW(map.encode(distant), std::logic_error);
  EXPECT_THROW(map.enable(), std::logic_error);
  map.observe(measured);
  EXPECT_EQ(map.encode(distant), measured);
  map.prepare();
  auto drifted = measured;
  drifted[0] += 0.001;
  map.observe(drifted);
  EXPECT_EQ(map.encode(distant), measured);
  EXPECT_EQ(map.reference(), measured);
  EXPECT_THROW(map.prepare(), std::logic_error);
}

TEST(GelloMapping, InvertsAllSevenDirectionsForNonzeroActivationPose) {
  Mapping map;
  Mapping::Joints reference{0.2, -0.7, 0.1, -2, 0.3, 1.5, -0.1};
  map.observe(reference);
  map.prepare();
  map.enable();
  // Repeated chunks and changing measured feedback must not rebase the map.
  for (double delta : {0.02, -0.04, 0.1}) {
    auto target = reference;
    for (size_t i = 0; i < 7; ++i) target[i] += delta * (i + 1);
    map.observe(target);
    const auto gello = map.encode(target);
    for (size_t i = 0; i < 7; ++i) {
      const auto controller_goal = reference[i] + Mapping::direction[i] * (gello[i] - reference[i]);
      EXPECT_NEAR(controller_goal, target[i], 1e-12);
      if (i == 0 || i == 1 || i == 6) EXPECT_NE(gello[i], target[i]);
      else EXPECT_DOUBLE_EQ(gello[i], target[i]);
    }
  }
  EXPECT_EQ(map.reference(), reference);
  EXPECT_THROW(map.prepare(), std::logic_error);
  EXPECT_THROW(map.enable(), std::logic_error);
}

TEST(GelloMapping, RejectsNonFiniteFeedbackAndTargets) {
  Mapping map;
  Mapping::Joints invalid{};
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(map.observe(invalid), std::invalid_argument);
  map.observe(Mapping::Joints{});
  EXPECT_THROW(map.encode(invalid), std::invalid_argument);
}
