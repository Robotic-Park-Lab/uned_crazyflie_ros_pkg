# uned_crazyflie_driver

`ament_python` package with the driver for communicating with **physical** Crazyflies via [`cflib`](https://github.com/Robotic-Park-Lab/crazyflie-lib-python) (lab fork), and with the common code base also used by `uned_crazyflie_webots` for the **virtual** (Webots) driver.

## Structure

- **`swarm_driver.py`** (entry point `swarm_driver`, the package's only node): `CFSwarmDriver`, manages a swarm of physical Crazyflies of size 1 or more. Instantiates one `Crazyflie_ROS2` per drone.
- **`crazyflie_agent.py`**: `Crazyflie_ROS2` class, the actual driver for each individual Crazyflie. Accepts physical hardware (`scf=`, via `cflib`) **or** Webots (`webots_node=`) — it's the same class that orchestrates physical mode today from `swarm_driver.py`; Webots mode is used by `uned_crazyflie_webots` through the common base below. Handles `config['type'] == 'digital_twin'` for the digital-twin case.
- **`webots_bootstrap.py`**: Webots motor/sensor initialization and the 12 cascaded `PIDController`s (position/velocity/attitude/angular rate) — extracted because it was **byte-for-byte identical** between `Crazyflie_ROS2.virtualCrazyflie()` and `uned_crazyflie_webots`'s driver. This is the real common base of the repo's two Webots drivers (`uned_crazyflie_webots/crazyflie_driver.py` and `crazyflie_driver_firmware.py`).
- **`agent.py`** / **`cmd_motion.py`**: `Agent` (neighbour tracking in formation, with RViz markers and `high_level_commander`) and `CMD_Motion` (motion command towards a target pose) classes, used by `crazyflie_agent.py`. These are **not** the same classes as their namesakes in `uned_crazyflie_missions` — there, the flight commands and tracking are deliberately simpler; see the comments in each file.
- **`pid_controller.py`** / **`pid_params.py`**: `PIDController` and `apply_controller_params()` (applies a `Pidcontroller` message to a controller, per axis). Shared by `crazyflie_agent.py`, `swarm_driver.py`, `uned_crazyflie_missions` and `uned_crazyflie_webots`.
- **`crazyflie_ros2_test.py`**: `Crazyflie_ROS2_TEST`, code that appears unused anywhere in the repo (see `AUDIT.md`) — moved here instead of deleted, pending Francisco's decision.

## Tests

Real unit tests for the code that is genuinely pure Python with no live ROS/`cflib`/Webots dependency, on top of the standard `ament_copyright`/`ament_flake8`/`ament_pep257` lint tests every package already has:

- **`test_pid_controller.py`** (`pid_controller.py`): the proportional/integral/derivative response, that `UpperLimit`/`LowerLimit` saturate the output (and that `UpperLimit == 0.0` disables saturation entirely — real behaviour of the code, not a bug), that the integral term uses the *previous* error rather than the one just set (also real, deliberate behaviour), the event-based trigger (`eval_threshold`) firing on a large enough delta and not re-firing right after, and the relay controller (`rele_update`) switching sign outside its band.
- **`test_pid_params.py`** (`pid_params.py`): `apply_controller_params()` against a fake `set_value` recorder and a fake message — verifies the right firmware parameter group/suffix per axis type (position continuous vs. event-based, velocity, attitude, rate), and in particular the real firmware quirks that are easy to break by refactoring: the `x`/`y` axes both mapping to the shared `xyVelMax` parameter, and rate axes (`droll`/`dpitch`/`dyaw`) mapping to a firmware parameter name with the leading `d` stripped (`roll_kp`, not `droll_kp`).
- **`test_cmd_motion.py`** (`cmd_motion.py`): `CMD_Motion.ckeck_pose()` (sic — real typo in the code, left as-is) leaves the pose untouched within its soft limit, only warns between the soft and hard limit, clamps to 85% of `xy_lim` (both signs) once past the hard limit, and never touches the Z axis (out of its scope).

**Deliberately not unit-tested**, and why: `Agent` (creates ROS subscriptions/publishers straight from `__init__`, and reads several fields off a live `parent` driver object — mocking that faithfully would be more fragile than valuable), `Crazyflie_ROS2` and `CFSwarmDriver` (full ROS nodes with a real dependency on `cflib`/Webots; testing them meaningfully needs a running simulator or hardware, not a unit test), and `webots_bootstrap.py` (needs a live Webots `Robot` object to initialize devices against). Testing these properly is real future work, not something to fake with a brittle mock just to show a passing test.

## Dependencies from other lab repos

`multi_agent_pkg` (from `RoboticPark`) and `cflib` (no rosdep key, install separately — see the lab fork linked above) must already be built/installed in the same workspace.
