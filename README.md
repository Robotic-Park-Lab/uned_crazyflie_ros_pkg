# uned_crazyflie_ros_pkg

> 📖 To understand this repo's branches and its contribution guide, see the [`doc`](https://github.com/Robotic-Park-Lab/uned_crazyflie_ros_pkg/tree/doc) branch.

ROS 2 packages and configuration files for teleoperating and simulating the Crazyflie 2.1 nano-drone in ROS 2, Webots and Matlab. The goal is a Hardware-in-the-Loop tool that is easy to scale and maintain, usable on its own by anyone who only wants to work with Crazyflies and ROS 2 — it does not require the rest of the [Robotic Park Lab](https://robotic-park-lab.github.io) setup, only the external dependencies listed below.

#### Structure
- **doc**. A `.tex` file going into more detail on the repository: ROS diagrams, bibliography, useful links, etc.
- **scripts**. Auxiliary files that are not part of any ROS package: `ros2 bag` post-processing (CSV conversion, plotting) and Matlab/Simulink models and identification scripts. See [scripts/README.md](scripts/README.md).
- **[uned_crazyflie_config](uned_crazyflie_config/README.md)**. ROS 2 package. Environment configuration: custom messages, 3D models, the unified `experience.launch.py` launch file, and RViz/RQT resources.
- **[uned_crazyflie_controllers](uned_crazyflie_controllers/README.md)**. ROS 2 package. Control nodes for different control architectures: Periodic PID (position and attitude/angular rate), Event-Based PID and Generalized Predictive Control (GPC) — meant as a teaching base for student work to add new techniques.
- **[uned_crazyflie_driver](uned_crazyflie_driver/README.md)**. ROS 2 package. Nodes that talk to Crazyflies (physical, and — via shared modules — virtual in Webots) through the `cflib` library: `swarm_driver`. Also hosts the Python code shared between this package, `uned_crazyflie_webots` and `uned_crazyflie_missions` (PID controller, PID parameter application) — previously a separate `uned_crazyflie_common` package, absorbed here as it wasn't worth keeping independent.
- **[uned_crazyflie_gui](uned_crazyflie_gui/README.md)**. ROS 2 package. PyQt graphical interface for handling a single robot, plus a generic RQT perspective and RViz file.
- **[uned_crazyflie_missions](uned_crazyflie_missions/README.md)**. ROS 2 package (formerly `uned_crazyflie_task`). High-level mission/task nodes solvable by one or several Crazyflies, indifferent to whether they are physical or virtual: formations (`leader_follower`, `shape_based_formation_control`, `formation_control_webots`) and TSP-style waypoint touring (`tsp_waypoints`).
- **[uned_crazyflie_webots](uned_crazyflie_webots/README.md)**. ROS 2 package. The two virtual Crazyflie 2.1 drivers in Webots (own controller, and real firmware/digital twin), plus the simulation worlds and models.

## Installation :book:

The target is [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) on **Ubuntu 22.04**. A Windows 10 setup is also supported for running the system on the PC that hosts the lab's [Vicon](https://www.vicon.com/) positioning system — Windows support is limited to the ROS 2 packages that don't need Webots (`uned_crazyflie_driver`, `uned_crazyflie_controllers`, `uned_crazyflie_gui`), since Webots simulation is only exercised on Ubuntu in this lab.

### Prerequisites 📋

##### ROS 2
Install ROS 2 Humble for your OS first, following the official documentation ([Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) / [Windows](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)).

##### Webots (Ubuntu only, required for `uned_crazyflie_webots`)
```
sudo apt install ros-humble-webots-ros2-driver
```
This installs [Webots](https://cyberbotics.com/) itself as a dependency. See [`uned_crazyflie_webots/README.md`](uned_crazyflie_webots/README.md) for the two simulated drivers (own controller vs. real Bitcraze firmware / digital twin) and what each needs.

##### Teleoperation (optional, for a joystick)
```
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
```
See [Joystick teleoperation](#joystick-teleoperation-) below.

##### Matlab
TO-DO — no fixed Matlab/Simulink version or toolbox list has been pinned down yet for `scripts/Matlab/`. See [scripts/README.md](scripts/README.md) for what each script does; verify manually the ones you plan to use.

##### Dependencies from other lab repositories
These are not declared with a rosdep key (they don't come from a public rosdep index) — clone and build them in the same workspace, alongside this repo:
- **Crazyflie Python library**: [Robotic-Park-Lab/crazyflie-lib-python](https://github.com/Robotic-Park-Lab/crazyflie-lib-python) (`master` branch), the lab's fork of `cflib`, required by `uned_crazyflie_driver`.
- **`multi_agent_pkg`**: from [Robotic-Park-Lab/RoboticPark](https://github.com/Robotic-Park-Lab/RoboticPark), required by `uned_crazyflie_driver` and `uned_crazyflie_webots` for multi-agent formation math (Lagrange multipliers for sphere/cone/ellipsoid geometries).
- **`vicon_receiver`** (optional, only if you actually have Vicon hardware): from [Robotic-Park-Lab/ros2-vicon-receiver](https://github.com/Robotic-Park-Lab/ros2-vicon-receiver).

##### Python dependencies
Installed automatically via `rosdep` below, but listed here for reference: `numpy`, `PyYAML`, `matplotlib`, `PyQt5`.

### Building the workspace

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_crazyflie_ros_pkg.git
git clone -b master https://github.com/Robotic-Park-Lab/crazyflie-lib-python.git
git clone -b humble-dev https://github.com/Robotic-Park-Lab/RoboticPark.git   # for multi_agent_pkg
cd ~/dev_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

On Windows, use `md \dev_ws\src` / `cd \dev_ws\src` instead, and `colcon build --merge-install` (symlink installs are not supported the same way on Windows).

## Usage 🔧

### Launching an experience

As of the modular restructuring (see `AUDIT.md` on the `doc` branch), there is a **single parametrized launch file**, `uned_crazyflie_config/launch/experience.launch.py`, instead of one `.launch.py` per demo. Each experience is a `.yaml` file in `uned_crazyflie_config/resources/`:

```
ros2 launch uned_crazyflie_config experience.launch.py config_file:=<experience>.yaml
```

| `config_file` | Description |
|---|---|
| `experience_swarm_teleop.yaml` | 2 virtual Crazyflies in Webots, teleoperated, with the generic RQT + RViz interface. |
| `experience_tsp_digital_twin.yaml` | 1 Crazyflie in digital-twin mode (real Bitcraze firmware driver), running the `tsp_waypoints` mission over 4 waypoints, with bag recording. |

More experiences will be added here as they are prepared — each one is just a new `.yaml` in `resources/`, following the schema documented at the top of `experience.launch.py` (`Simulation` / `Robots` / `Interface` / `Data_Logging` / `Missions`), no new launch file needed. The 14+6 legacy per-demo `.launch.py` files (in `uned_crazyflie_config/launch/` and `uned_crazyflie_webots/launch/`) are still present pending manual review/retirement — see `AUDIT.md`.

### Joystick teleoperation 🎮

[`teleop_twist_joy`](https://github.com/ros2/teleop_twist_joy) publishes a `geometry_msgs/Twist` on `/cmd_vel` from a joystick. Both Webots drivers in `uned_crazyflie_webots` (`crazyflie_driver.py` and `crazyflie_driver_firmware.py`) already subscribe to `<robot_id>/cmd_vel`, so remapping is all that's needed for a **simulated** Crazyflie:

```
ros2 run joy joy_node
ros2 run teleop_twist_joy teleop_node --ros-args -r cmd_vel:=/dron01/cmd_vel
```

Run this alongside an `experience.launch.py` that has `dron01` as a `virtual`/`digital_twin` robot. Tune `teleop_twist_joy`'s own parameters (axis mapping, scale) for your controller — see its [documentation](https://index.ros.org/p/teleop_twist_joy/).

**Known gap**: the physical-hardware driver (`Crazyflie_ROS2` in `uned_crazyflie_driver/crazyflie_agent.py`) does **not** subscribe to a `cmd_vel` topic today — joystick teleoperation only works against a Webots-simulated Crazyflie right now, not real hardware. Wiring it up for physical drones (via `cflib`'s offboard `Twist`-style setpoints) is open work, not yet done.

### Crazyflie orders

Regardless of the mission/task running, take-off and landing are commanded with a `std_msgs/String` on `<robot_id>/order`:
```
ros2 topic pub /dron01/order std_msgs/String "{data: 'take_off'}"
ros2 topic pub /dron01/order std_msgs/String "{data: 'land'}"
```

### Firmware log variables

https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#pm

![Alt text](doc/figs/rosgraph_ROS2.png?raw=true "rqt_graph")

> This `rqt_graph` capture predates the modular restructuring (topic/node names have since changed) — kept for illustration until a fresh one is captured.

### Matlab controller
TO-DO

### Hardware-in-the-Loop
TO-DO

## Authors ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Related publications :paperclip:
- Mañas-Álvarez, F.J., Guinaldo, M., Dormido, R., Socas, R., Dormido, S. Control basado en eventos mediante umbral relativo aplicado al control de altitud de cuadricópteros Crazyflie 2.1. In XLII Jornadas de Automática: libro de actas. Castelló, September 1-3, 2021 (pp. 341-348). Chapter DOI: https://doi.org/10.17979/spudc.9788497498043.341 Book DOI: https://doi.org/10.17979/spudc.9788497498043
