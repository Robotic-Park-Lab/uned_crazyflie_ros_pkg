# uned_crazyflie_config

`ament_cmake` package with every configuration element shared by the rest of the packages in this repo: custom messages, 3D models, launch files, and tool configuration (RViz, RQT).

## Structure

- **`msg/`**: custom ROS 2 messages.
  - `Pidcontroller.msg`: PID controller parameters per axis (`id`, `kp`/`ki`/`kd`/`td`/`nd`, `co`/`ai` for event-based mode, limits).
  - `StateEstimate.msg`: state estimated by the Crazyflie (position, attitude, thrust).
  - `Cmdsignal.msg`: low-level command signal (`thrust`, `roll`, `pitch`, `yaw`, `vbat`).
  - `Triggering.msg`, `Actuators.msg`: event triggering and actuators (the latter inherited from `mav_msgs`, for compatibility with the rest of the MAV ecosystem).
- **`model/`**: 3D models of the Crazyflie (URDF/meshes), used by RViz.
- **`resources/crazyflie.urdf`**: the Webots robot description for a virtual Crazyflie, loading `uned_crazyflie_driver`'s `webots_driver` plugin. Placeholders (`CameraAlwayOn`/`CameraEnable`/`CameraUpdateRate`, `name_id_value`, `config_file_path`) are substituted by `experience.launch.py` at launch time.
- **`worlds/`**: the Webots simulation worlds (`RoboticPark_N01.wbt`...`N05.wbt`) referenced by experience `.yaml` files' `Operation.world`.
- **`launch/experience.launch.py`**: single launch file parameterized by experience — one `ros2 launch uned_crazyflie_config experience.launch.py config_file:=<experience>.yaml` instead of one `.launch.py` per demo. Reads the `Operation`/`Robots`/`Interface`/`Data_Logging`/`Missions` sections from a `.yaml` in `resources/`; see the file's own header for the full schema. This is now the **only** launch file in the repo — the old per-demo `.launch.py` files, and the standalone `uned_crazyflie_webots` package they used to live alongside, have been removed. Francisco is adding more example experience `.yaml` files incrementally.
- **`resources/`**: the current experience `.yaml` files (`demo_individual_teleop_webots.yaml`, `demo_individual_teleop_vicon.yaml`, `demo_individual_waypoints_webots.yaml`) plus the config files they reference (`demo_individual_waypoints.yaml`, `demo_individual_waypoints_topics.yaml` for `uned_crazyflie_missions`; `crazyflie_parameters.yaml`, `crazyflie_distances.yaml`, `LightHouseV2_Geometry.yaml`).
- **`rviz/demo_individual.rviz`**: the RViz configuration used by the current experience files (`rviz/demo_formation_N20.rviz` is a leftover from an older demo, pending review). For a generic `.rviz` that works for any Crazyflie, see `uned_crazyflie_gui/rviz/crazyflie.rviz`.
- **`rqt/`**: RQT perspectives from older demos (`Robotic Park Lab.perspective`, `Swarm_teleop_one.perspective`), not referenced by any current experience `.yaml` — pending review. For a generic RQT perspective, see `uned_crazyflie_gui/rqt/crazyflie.perspective`.

## Tests

This package has no nodes of its own (only messages, resources and launch files), so there isn't much to unit-test beyond what `rosidl` already enforces at build time (a build failure if a `.msg` is malformed). The one thing that genuinely needed a real test was `experience.launch.py`, since it contains real branching logic (which nodes to launch depending on `Operation`/`Robots`/`Interface`/`Data_Logging`/`Missions`) that a build failure would never catch:

- `test/test_experience_launch.py` (via `ament_cmake_pytest`): loads the **installed** `experience.launch.py` and runs its `get_ros2_nodes()` with a real `LaunchContext` against the 3 current experience files, asserting the actions it generates are the right ones — e.g. that `demo_individual_teleop_webots.yaml` adds a `WebotsLauncher` + a `WebotsController` + `rviz2` and no `uned_crazyflie_driver` node (no physical robot in it), that `demo_individual_teleop_vicon.yaml` adds the `vicon_receiver` node instead, and that `demo_individual_waypoints_webots.yaml` launches both `uned_crazyflie_missions` mission nodes it configures. It also checks that the resources it depends on (`uned_crazyflie_gui`'s generic RQT perspective and RViz file, the example `.yaml`s and `crazyflie.urdf` themselves) actually get installed where the launch file expects them.
