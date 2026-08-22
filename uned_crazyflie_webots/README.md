# uned_crazyflie_webots

`ament_python` package with the **virtual** Crazyflie 2.1 drivers in [Webots](https://cyberbotics.com/), plus the simulation worlds/meshes.

## The two drivers

`webots_ros2_driver` does **not** load these drivers with `ros2 run`: it instantiates them at simulation time by class path, from the `<plugin type="...">` tag of the robot's URDF (`resources/crazyflie.urdf` / `crazyflie_firmware.urdf`). That is why this package has no `console_scripts` in `setup.py`.

- **`crazyflie_driver.py`** (`CrazyflieWebotsDriver`, loaded from `resources/crazyflie.urdf` and `crazyflie_IPC.urdf`): custom geometric controller in Python — distance/pose-based formations, with ML1/ML2/ML3 variants and sphere/cone/ellipsoid geometries. This is the active driver in the 6 demos under `launch/`.
- **`crazyflie_driver_firmware.py`** (`CrazyflieWebotsDriver`, loaded from `resources/crazyflie_firmware.urdf`, new): uses `cffirmware`, Bitcraze's real firmware compiled for software-in-the-loop — the **digital twin** variant: the same control code that flies on real hardware, instead of a Python reimplementation. Requires `cffirmware` built separately (`make bindings_python` in [`crazyflie-firmware`](https://github.com/Robotic-Park-Lab/crazyflie-firmware)) and the `CRAZYFLIE_FIRMWARE_PATH` environment variable — not yet wired from any demo `.launch.py`, only from its URDF (and from `experience.launch.py`'s `driver: firmware` option, see `uned_crazyflie_config`).

Both share with `uned_crazyflie_driver` the motor/sensor bootstrap and the 12 cascaded `PIDController`s (`uned_crazyflie_driver.webots_bootstrap`), neighbor tracking (`uned_crazyflie_driver.agent.Agent`) and `PIDController` (`uned_crazyflie_driver.pid_controller`). The formation-control logic itself is **not** unified with `Crazyflie_ROS2` (`uned_crazyflie_driver/crazyflie_agent.py`, the physical Crazyflie's driver) — they are real, divergent gradient algorithms between hardware and simulation; see `AUDIT.md` (`doc` branch) for the detail of why they were kept separate.

## Structure

- **`resources/`**: robot URDF/models for Webots.
- **`worlds/`**: simulation `.wbt` worlds (1 to 4 Crazyflies, spherical formation) and their meshes.
- **`controllers/`**: native Webots C/Python controllers (simulator/Bitcraze templates), excluded from lint since they are generated/vendored code.
- **`launch/`**: 6 demo `.launch.py` files, partially duplicated with the ones in `uned_crazyflie_config`. The single launch (`uned_crazyflie_config/launch/experience.launch.py`) already covers this case — pending manual review by Francisco to decide which of these 6 get retired (see `AUDIT.md`, Phase 2 point 8).

## Tests

Beyond the standard lint tests (copyright/flake8/pep257, excluding `controllers/`), this package has **no functional tests of its own**, and honestly can't have many without real Webots installed:

- `crazyflie_driver.py`/`crazyflie_driver_firmware.py`'s `init()`/`step()` and the formation-gradient methods (`distance_gradient_controller`/`pose_gradient_controller`) are tightly coupled to a real Webots `Robot` object (`robot.getDevice(...)`, `robot.getTime()`, ...) — not reasonably testable here without Webots installed (it isn't, in this sandbox), and not worth mocking to the point of testing a fake instead of the real thing.
- The one piece that genuinely is pure logic and shared with this package — `init_webots_cascade_controllers()` in `uned_crazyflie_driver/webots_bootstrap.py` (just returns 12 `PIDController` instances with fixed gains, no Webots dependency) — is tested in `uned_crazyflie_driver/test/`, not here, since it lives in that package.

## Dependencies on other lab repos

`multi_agent_pkg` (from `RoboticPark`) must already be built in the same workspace.
