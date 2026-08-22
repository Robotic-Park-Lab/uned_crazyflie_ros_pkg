# uned_crazyflie_config

`ament_cmake` package with every configuration element shared by the rest of the packages in this repo: custom messages, 3D models, launch files, and tool configuration (RViz, RQT).

## Structure

- **`msg/`**: custom ROS 2 messages.
  - `Pidcontroller.msg`: PID controller parameters per axis (`id`, `kp`/`ki`/`kd`/`td`/`nd`, `co`/`ai` for event-based mode, limits).
  - `StateEstimate.msg`: state estimated by the Crazyflie (position, attitude, thrust).
  - `Cmdsignal.msg`: low-level command signal (`thrust`, `roll`, `pitch`, `yaw`, `vbat`).
  - `Triggering.msg`, `Actuators.msg`: event triggering and actuators (the latter inherited from `mav_msgs`, for compatibility with the rest of the MAV ecosystem).
- **`model/`**: 3D models of the Crazyflie (URDF/Xacro/SDF/`.dae` meshes), used both in RViz and Webots. `model/convert/` keeps the intermediate files from the Gazebo→Webots conversion.
- **`launch/experience.launch.py`** (new): single launch file parameterized by experience — one `ros2 launch uned_crazyflie_config experience.launch.py config_file:=<experience>.yaml` instead of one `.launch.py` per demo. Reads the `Simulation`/`Robots`/`Interface`/`Data_Logging`/`Missions` sections from a `.yaml` in `resources/`; see the file's own header for the full schema, and `resources/experience_swarm_teleop.yaml` (teleoperation, 2 virtual drones) / `resources/experience_tsp_digital_twin.yaml` (1 drone in digital-twin mode with the real firmware driver, running the `tsp_waypoints` mission) as real examples, verified by actually running the launch logic (not just reading it).
- **`launch/*.launch.py`** (the previous 14): **pending manual review by Francisco** — decide, demo by demo, which ones get retired once migrated to an experience `.yaml` and which ones stay. See `AUDIT.md` (`doc` branch), Phase 2 point 8.
- **`resources/`**: one `.yaml` configuration file per demo (default PID parameters, formation geometry, waypoints, Lighthouse V2 geometry for Vicon...), plus the new experience `.yaml` files (`experience_*.yaml`) for `experience.launch.py`.
- **`rviz/`**: `.rviz` files tied to specific demos (`demo_swarm_formation.rviz`, `demo_swarm_teleop.rviz`, `sphere.rviz`, `test.rviz`). For a generic `.rviz` that works for any Crazyflie, see `uned_crazyflie_gui/rviz/crazyflie.rviz`.
- **`rqt/`**, **`Webots_rviz.perspective`**: RQT perspectives tied to specific demos. For a generic RQT perspective, see `uned_crazyflie_gui/rqt/crazyflie.perspective`.

## Dependency on other lab repos

Some `.launch.py` files in this package reference a fixed IP (`10.196.92.136`, the Vicon system's PC) — deliberately left as-is until it can be evaluated against the real lab network (see `AUDIT.md`).

## Tests

This package has no nodes of its own (only messages, resources and launch files), so there isn't much to unit-test beyond what `rosidl` already enforces at build time (a build failure if a `.msg` is malformed). The one thing that genuinely needed a real test was the new `experience.launch.py`, since it contains real branching logic (which nodes to launch depending on `Simulation`/`Robots`/`Interface`/`Data_Logging`/`Missions`) that a build failure would never catch:

- `test/test_experience_launch.py` (via `ament_cmake_pytest`): loads the **installed** `experience.launch.py` and runs its `get_ros2_nodes()` with a real `LaunchContext`, for both example experience files, asserting the actions it generates are the right ones — e.g. that `experience_swarm_teleop.yaml` launches exactly 2 `webots_ros2_driver`/`driver` nodes and no `swarm_driver` (no physical/digital_twin robot in it), and that `experience_tsp_digital_twin.yaml` launches both `swarm_driver` *and* the robot's Webots twin for its `digital_twin` robot, plus the `tsp_waypoints` mission node and a `ros2 bag record` process. It also checks that the resources it depends on (`uned_crazyflie_gui`'s generic RQT perspective and RViz file, the example `.yaml`s themselves) actually get installed where the launch file expects them.
