// Exercises the real, compiled PID math of periodic_pid_attituderate_controller.cpp
// (AttitudeRateController::pid_controller / init_controller) through a
// friend test fixture. Real behavior verified from reading the source
// directly: unlike both position controllers, this one (a) saturates
// unconditionally (no "if upperlimit != 0.0" guard) and (b) applies an
// anti-windup correction to the integral term whenever it saturates.

#include <cmath>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "uned_crazyflie_controllers/CrazyflieAttitudeRateController.hpp"

// gtest's TEST_F(AttitudeRateControllerTest, Name) expands to a class
// deriving from AttitudeRateControllerTest, and C++ friendship is NOT
// inherited -- so the private members must be reached through wrapper
// methods defined directly on this (friend) fixture class, not called
// straight from the TEST_F bodies.
class AttitudeRateControllerTest : public ::testing::Test
{
protected:
  std::shared_ptr<AttitudeRateController> node;

  void SetUp() override
  {
    node = std::make_shared<AttitudeRateController>();
  }

  pid_s InitController(
    const char id[], double kp, double ki, double kd, double td, int nd,
    double upperlimit, double lowerlimit)
  {
    return node->init_controller(id, kp, ki, kd, td, nd, upperlimit, lowerlimit);
  }

  double RunPid(pid_s & controller, double dt)
  {
    return node->pid_controller(controller, dt);
  }
};

TEST_F(AttitudeRateControllerTest, ProportionalOnlyMatchesKpTimesError)
{
  // Ki=0 disables the anti-windup division (kp/ki), Kd=0 keeps derivative
  // at 0 regardless of td/nd. Upper/lower limits set permissive enough
  // (this file clamps unconditionally, even at 0.0) so the raw
  // proportional term is not clamped.
  auto controller = InitController("Pitch", 2.0, 0.0, 0.0, 1.0, 100, 100.0, -100.0);
  controller.error[0] = 3.0;
  double out = RunPid(controller, 0.01);
  EXPECT_DOUBLE_EQ(out, 6.0);
}

TEST_F(AttitudeRateControllerTest, SaturatesEvenWhenUpperLimitIsZero)
{
  // Real, verified difference from the position controllers: no
  // "if (upperlimit != 0.0)" guard here, so upperlimit=0.0 still clamps.
  auto controller = InitController("Pitch", 100.0, 0.0, 0.0, 1.0, 100, 0.0, 0.0);
  controller.error[0] = 1.0;  // Kp * error = 100
  double out = RunPid(controller, 0.01);
  EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST_F(AttitudeRateControllerTest, AntiWindupCorrectsIntegralOnSaturation)
{
  // Kp=10, Ki=1, Kd=0, upperlimit=2, lowerlimit=-2, dt=0.1, error[0]=1.
  // out_i (pre-clamp) = 10, clamped to 2 -> integral -= (2 - 10) * sqrt(10/1).
  auto controller = InitController("Pitch", 10.0, 1.0, 0.0, 1.0, 100, 2.0, -2.0);
  controller.error[0] = 1.0;
  double out = RunPid(controller, 0.1);

  EXPECT_DOUBLE_EQ(out, 2.0);
  double expected_integral = 0.0 - (2.0 - 10.0) * std::sqrt(10.0 / 1.0);
  EXPECT_NEAR(controller.integral, expected_integral, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
