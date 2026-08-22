// Exercises the real, compiled PID math of eventbased_pid_position_controller.cpp
// (PositionController::pid_controller / init_controller) through a friend
// test fixture. Same PositionController class/header as
// periodic_pid_position_controller.cpp, but a DIFFERENT compiled definition
// of pid_controller() in this translation unit -- notably it sets the
// member `events = true` on every call (checked below), unlike the
// periodic version.

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

  bool Events()
  {
    return node->events;
  }
};

TEST_F(PositionControllerTest, ProportionalOnlyMatchesKpTimesError)
{
  auto controller = InitController("Z", 2.0, 0.0, 0.0, 1.0, 0, 0.0, 0.0);
  controller.error[0] = 3.0;
  double out = RunPid(controller, 0.01);
  EXPECT_DOUBLE_EQ(out, 6.0);
}

TEST_F(PositionControllerTest, SaturatesToUpperAndLowerLimit)
{
  auto controller = InitController("Z", 100.0, 0.0, 0.0, 1.0, 0, 5.0, -5.0);
  controller.error[0] = 1.0;
  double out = RunPid(controller, 0.01);
  EXPECT_DOUBLE_EQ(out, 5.0);
}

TEST_F(PositionControllerTest, MarksEventsTrueOnEveryCall)
{
  ASSERT_FALSE(Events());  // default from the header
  auto controller = InitController("Z", 1.0, 0.0, 0.0, 1.0, 0, 0.0, 0.0);
  controller.error[0] = 0.1;
  RunPid(controller, 0.01);
  EXPECT_TRUE(Events());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
