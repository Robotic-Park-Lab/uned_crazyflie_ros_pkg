# uned_crazyflie_controllers

`ament_cmake` package (C++) with the Crazyflie 2.1's flight controllers, designed as a teaching base: each controller is an independent `rclcpp::Node`, meant so that student work can add new techniques following the same pattern.

## Included controllers

| Executable | Class / header | Architecture |
|---|---|---|
| `periodic_pid_position_controller` | `PositionController` / `CrazyfliePositionController.hpp` | Periodic PID position control (X/Y/Z) |
| `eventbased_pid_position_controller` | `PositionController` / `CrazyfliePositionController.hpp` | Event-based PID position control (relative threshold, see the Mañas-Álvarez et al. publication cited in the root `README.md`) |
| `periodic_pid_attituderate_controller` | `AttitudeRateController` / `CrazyflieAttitudeRateController.hpp` | Periodic PID attitude (roll/pitch/yaw) and angular rate control |
| `generalized_predictive_controller` | (same base as `PositionController`) / `CrazyfliePositionController.hpp` | Generalized Predictive Control (GPC) of position |

Each controller reads its gains from ROS parameters (`ZKp`/`ZKi`/`ZKd`/..., `PitchKp`/..., depending on the axis/controller) and publishes/subscribes on the topics and custom messages of `uned_crazyflie_config` (`Pidcontroller`, `StateEstimate`).

Note: `periodic_pid_position_controller` and `eventbased_pid_position_controller` both build the *same* `PositionController` class declared in `CrazyfliePositionController.hpp`, but each `.cpp` file provides its own definition of that class's methods (`pid_controller()`, `initialize()`, ...) and is compiled into its own standalone executable — they are never linked together. `generalized_predictive_controller` also includes that header but does not define/use `pid_controller()` at all; it solves the position loop through its own GPC state-space matrices instead.

## Tests

Until now this package only had the standard ROS 2 lint tests (`ament_lint_auto`/`ament_lint_common`) — no functional coverage of the actual PID math. Three `ament_add_gtest` targets were added under `test/`, each exercising the *real, compiled* `pid_controller()`/`init_controller()` of one specific `.cpp` file — nothing is reimplemented or duplicated in the tests:

- `test_periodic_pid_position_controller.cpp` — compiles and links `src/periodic_pid_position_controller.cpp` itself. Verifies gain copying in `init_controller()`, that the proportional term matches `Kp * error` on the first call, that the integral term accumulates correctly across calls, that the output saturates to `[lowerlimit, upperlimit]`, and — a real, verified quirk of this specific file — that saturation is skipped entirely when `upperlimit == 0.0` (`if (controller.upperlimit != 0.0)` in the source).
- `test_eventbased_pid_position_controller.cpp` — same `PositionController` class, but links `src/eventbased_pid_position_controller.cpp`, whose `pid_controller()` is a *different* compiled definition: it always sets the member `events = true` on every call, verified explicitly.
- `test_attituderate_controller.cpp` — links `src/periodic_pid_attituderate_controller.cpp` (`AttitudeRateController` class). Verifies two real, verified differences from the position controllers: saturation is unconditional here (clamps even when `upperlimit == 0.0`, unlike the two files above), and it applies an anti-windup correction to the integral term whenever the output saturates (`integral -= (out - out_i) * sqrt(kp / ki)`) — the test computes the expected corrected integral by hand and checks it matches exactly.

### How this was made testable without changing behavior

`pid_controller()`/`init_controller()` are private methods of `PositionController`/`AttitudeRateController`, and each `.cpp` file also defines its own `int main()` that would collide with gtest's own `main()` if linked into a test binary. Two small, additive changes made the *existing, unmodified* PID logic testable in isolation:

1. Each file's `main()` is now wrapped in `#ifndef UNED_CRAZYFLIE_CONTROLLERS_TEST_BUILD` / `#endif`. Normal builds are unaffected (the macro is undefined); only the test targets define it, via `target_compile_definitions`, to exclude `main()` and link the rest of the file into the gtest binary instead.
2. Each header adds a single `friend class <Name>Test;` declaration for the corresponding gtest fixture, so the tests can call the private methods directly — no method visibility was changed for the actual node executables.

No PID math, saturation logic, or anti-windup behavior was altered anywhere in this pass; only test scaffolding was added.

### What is intentionally not covered

`initialize()`/`iterate()` (parameter reading, publisher/subscriber wiring, the ROS spin loop) and `generalized_predictive_controller`'s GPC state-space logic are not unit tested here — they need a running node graph or non-trivial state-space fixtures to exercise meaningfully, and forcing a shallow test around them would not verify anything real. `eval_threshold()`/`init_triggering()` (the event-based noise/threshold logic in `CrazyfliePositionController.hpp`) are also untested for the same reason: not pure enough to test cheaply without a larger effort than this pass covers.

## Adding a new control technique

1. New header at `include/uned_crazyflie_controllers/Crazyflie<Name>Controller.hpp`, inheriting from `rclcpp::Node` like the existing ones.
2. New source at `src/<name>_controller.cpp` implementing `initialize()`/`iterate()` (or an equivalent pattern).
3. Register the executable in `CMakeLists.txt` (`add_executable` + `ament_target_dependencies` + add it to `install(TARGETS ...)`).
4. If the new controller has PID-like pure logic worth testing, follow the pattern above: `#ifndef`-guard `main()`, add a `friend class` test fixture, and a new `ament_add_gtest` target.

## Dependencies

`rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `Eigen3` (via `eigen3_cmake_module`), and the custom messages from `uned_crazyflie_config`. Tests additionally require `ament_cmake_gtest`.
