# uned_crazyflie_missions

`ament_python` package (formerly `uned_crazyflie_task`) with **high-level mission** nodes for one or several Crazyflies: formations and waypoint tours. All of them are agnostic to whether the drone is physical or virtual (Webots) — they talk to the driver over topics, not directly to `cflib` or Webots (except `leader_follower.py`/`shape_based_formation_control.py`, see below).

## Nodes

- **`tsp_waypoints`**: visits a set of waypoints, solving the visiting order as a traveling salesman problem (TSP) with a nearest-neighbor + 2-opt heuristic (`tsp.py`, tested in `test/test_tsp.py` with no ROS dependency). Works the same with a physical or virtual Crazyflie: it only uses `<robot_id>/order` (`take_off`/`land`) and `<robot_id>/goal_pose` as commands, reading `<robot_id>/local_pose` as feedback — the same topic contract already exposed by `uned_crazyflie_driver`/`uned_crazyflie_webots`. Parameters: `robot_id` (drone namespace, default `dron01`), `waypoints` (flat list `[x1,y1,z1, x2,y2,z2, ...]`), `tolerance` (meters to consider a waypoint reached), `loop` (repeat the tour).
  ```
  ros2 run uned_crazyflie_missions tsp_waypoints --ros-args \
    -p robot_id:=dron01 \
    -p waypoints:="[0.0,0.0,1.0, 2.0,0.0,1.0, 2.0,2.0,1.0, 0.0,2.0,1.0]"
  ```
- **`formation_control_webots`**: formation control purely over topics (`<id>/goal_pose`, `<id>/pose`), with no `cflib` dependency — the most "driver-agnostic" of the three formation nodes.
- **`leader_follower`** / **`shape_based_formation_control`**: leader-follower formation and shape-based formation. Unlike the two nodes above, these two connect **directly** to `cflib.crazyflie.swarm.Swarm` (physical hardware only) instead of talking to the driver over topics — a different architecture within the same package, inherited from the original code; not touched in this pass.

## Adding a new mission

Follow the same pattern as `tsp_waypoints`: a node that publishes `<robot_id>/order`/`<robot_id>/goal_pose` and subscribes to `<robot_id>/local_pose`, without talking directly to `cflib`/Webots, so that it works the same with a physical or virtual drone. Register it as a `console_script` in `setup.py`.

## Ideas for future missions (not implemented)

Autonomous navigation and exploration, also mentioned as a goal for this package, have not been implemented in this pass — unlike the TSP case (which Francisco explicitly requested over "a set of points"), there is no concrete specification of which algorithm/sensor to use, and adding an empty node just to check a box would not add anything real.

## Tests

Beyond the standard lint tests (`ament_copyright`, `ament_flake8`, `ament_pep257`), this package has real unit tests for the logic that is pure enough to test without a running ROS graph:

- **`test_tsp.py`** (4 tests) exercises `tsp.py` directly: `nearest_neighbor` visits every point exactly once, `two_opt` never makes a tour worse, `solve_tsp` finds the true optimum on a 4-point square (the perimeter, not a diagonal crossing), and the 2-point edge case is handled.
- **`test_cmd_motion.py`** (8 tests) exercises `CMD_Motion.ckeck_pose()` (position saturation) with a fake logger: positions inside `xy_warn` are left untouched, positions between `xy_warn` and `xy_lim` only warn, positions beyond `xy_lim` get clamped to 95% of `xy_warn` with the correct sign, and X/Y are clamped independently. It also exercises `send_pose_data_`/`send_offboard_setpoint_` with a fake Crazyflie object recording calls, including the `relative_pose` branch introduced when `leader_follower.py` and `shape_based_formation_control.py` were deduplicated (see git history) — the default call is asserted to reproduce the exact pre-refactor `leader_follower.py` behaviour.
- **`test_agent.py`** (4 tests) exercises `Agent` with a minimal fake `parent` object (duck-typed `get_logger()`/`create_subscription()`, no real ROS node needed): the constructor stores id/position and subscribes to the right `<id>/pose` topic, `str_()` includes the id and coordinates, and `gtpose_callback` stores the received pose.

**Deliberately not unit-tested**: `tsp_waypoints.py`'s `TSPWaypointsNode`, `formation_control_webots.py`, `leader_follower.py`, and `shape_based_formation_control.py` are full ROS nodes (or, for the latter two, also talk directly to `cflib.crazyflie.swarm.Swarm`) — wiring a meaningful unit test for them would need a running ROS graph or a real/mocked Crazyflie swarm, which is out of scope for this pass. `tsp_waypoints.py`'s state machine was instead checked with a live `ros2 run` integration test against a harness node during development (see `AUDIT.md` on the `doc` branch for that verification).
