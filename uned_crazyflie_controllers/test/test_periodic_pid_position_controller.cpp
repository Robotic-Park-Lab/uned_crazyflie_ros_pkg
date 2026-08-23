// Exercises the real, compiled PID math of periodic_pid_position_controller.cpp
// (PositionController::pid_controller / init_controller) through a friend
// test fixture -- no logic is duplicated or reimplemented here.
//
// Real behavior verified from reading the source directly (not assumed):
//  - Saturates to [lowerlimit, upperlimit] only when upperlimit != 0.0.
//  - No anti-windup correction on the integral term (that line is commented
//    out in this specific file -- periodic_pid_attituderate_controller.cpp
//    does apply it, tested separately).

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "uned_crazyflie_controllers/CrazyfliePositionController.hpp"

// gtest's TEST_F(PositionControllerTest, Name) expands to a class deriving
// from PositionControllerTest, and C++ friendship is NOT inherited -- so
// the private members must be reached through wrapper methods defined
// directly on this (friend) fixture class, not called straight from the
// TEST_F bodies.
class PositionControllerTest : public ::testing::Test
{
protected:
  std::shared_ptr<PositionController> node;

  void SetUp() override
  {
    node = std::make_shared<PositionController>();
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

TEST_F(PositionControllerTest, InitControllerCopiesGains)
{
  auto controller = InitController("Z", 1.5, 0.5, 0.1, 0.02, 100, 10.0, -10.0);
  EXPECT_DOUBLE_EQ(controller.kp, 1.5);
  EXPECT_DOUBLE_EQ(controller.ki, 0.5);
  EXPECT_DOUBLE_EQ(controller.kd, 0.1);
  EXPECT_DOUBLE_EQ(controller.td, 0.02);
  EXPECT_EQ(controller.nd, 100);
  EXPECT_DOUBLE_EQ(controller.upperlimit, 10.0);
  EXPECT_DOUBLE_EQ(controller.lowerlimit, -10.0);
  EXPECT_DOUBLE_EQ(controller.integral, 0.0);
  EXPECT_DOUBLE_EQ(controller.error[0], 0.0);
  EXPECT_DOUBLE_EQ(controller.error[1], 0.0);
}

TEST_F(PositionControllerTest, ProportionalOnlyMatchesKpTimesError)
{
  // Kp only (Ki=Kd=0), first call: integral/derivative terms are still 0,
  // so the output must equal exactly Kp * error[0].
  auto controller = InitController("Z", 2.0, 0.0, 0.0, 1.0, 0, 0.0, 0.0);
  controller.error[0] = 3.0;
  double out = RunPid(controller, 0.01);
  EXPECT_DOUBLE_EQ(out, 6.0);
}

TEST_F(PositionControllerTest, IntegralAccumulatesAcrossCalls)
{
  // Ki only: after N calls with a constant error, the integral term grows
  // by Ki * error * dt each call (error[1] lags error[0] by one call, so
  // the first call integrates 0 -- matches the real update order).
  auto controller = InitController("Z", 0.0, 1.0, 0.0, 1.0, 0, 0.0, 0.0);
  controller.error[0] = 2.0;
  double dt = 0.1;

  double out1 = RunPid(controller, dt);
  EXPECT_DOUBLE_EQ(out1, 0.0);  // error[1] was still 0 on the first call

  controller.error[0] = 2.0;  // simulate the same constant error next tick
  double out2 = RunPid(controller, dt);
  EXPECT_DOUBLE_EQ(out2, 1.0 * 2.0 * dt);  // Ki * error[1](=2.0) * dt
}

TEST_F(PositionControllerTest, SaturatesToUpperAndLowerLimit)
{
  auto controller = InitController("Z", 100.0, 0.0, 0.0, 1.0, 0, 5.0, -5.0);
  controller.error[0] = 1.0;  // Kp * error = 100, way above upperlimit
  double out = RunPid(controller, 0.01);
  EXPECT_DOUBLE_EQ(out, 5.0);

  controller.error[0] = -1.0;  // -100, way below lowerlimit
  double out2 = RunPid(controller, 0.01);
  EXPECT_DOUBLE_EQ(out2, -5.0);
}

TEST_F(PositionControllerTest, ZeroUpperLimitDisablesSaturation)
{
  // Real behavior of this file: saturation only applies "if(upperlimit != 0.0)".
  auto controller = InitController("Z", 100.0, 0.0, 0.0, 1.0, 0, 0.0, 0.0);
  controller.error[0] = 1.0;
  double out = RunPid(controller, 0.01);
  EXPECT_DOUBLE_EQ(out, 100.0);  // not clamped
}

TEST_F(PositionControllerTest, ErrorHistoryShiftsAfterEachCall)
{
  auto controller = InitController("Z", 0.0, 0.0, 0.0, 1.0, 0, 0.0, 0.0);
  controller.error[0] = 7.0;
  RunPid(controller, 0.01);
  EXPECT_DOUBLE_EQ(controller.error[1], 7.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
