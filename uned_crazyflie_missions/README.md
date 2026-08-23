# uned_crazyflie_missions

`ament_python` package with **high-level mission** nodes for one or several Crazyflies. Every node here talks **only over topics** — never directly to `cflib` or Webots — so the same node works unchanged with a physical or a virtual drone, and never needs to import anything from `uned_crazyflie_driver`. Each node is configured from a `.yaml` (see `uned_crazyflie_config/resources/`), not from hardcoded values or a pile of ROS parameters, so a new experience only needs a new config file, not new code.

This is a from-scratch redesign (2026-08-22): the previous version of this package (`leader_follower.py`, `shape_based_formation_control.py`, `formation_control_webots.py`, plus their `agent.py`/`cmd_motion.py` support classes) connected **directly** to `cflib.crazyflie.swarm.Swarm` — three separate, duplicate, obsolete driver implementations living inside what was supposed to be a driver-agnostic package, predating `uned_crazyflie_driver`'s `swarm_driver`/`webots_driver` consolidation. All of that is gone; the real formation-control law from `shape_based_formation_control.py` was extracted into `formation.py` below, everything else was dead weight.

## Nodes

- **`formation`** (`formation.py`): one instance per robot. Subscribes to its own pose and its neighbours' poses, publishes its own absolute goal pose, using a distance/offset consensus law (mean neighbour error + integral term — see the module docstring for the exact math and where it came from). Config:
  ```yaml
  config:
    output: '/dron01/goal_pose'
    input: '/dron01/local_pose'
    period: 0.02
  neighbours:
    N0: {id: dron02, topic: '/dron02/local_pose', dx: 0.25, dy: 0.25, dz: 0.1}
  gains:
    integral_divisor_xy: 1.0
    integral_divisor_z: 5.0
  ```
  **This config schema is this package's own proposal** — Francisco described the node's behaviour ("subscribe to robot positions, publish goal_pose") but didn't hand over a reference `.yaml` for it the way he did for the two nodes below, so there's no example file to point at yet. Flagging for review, not silently assumed.
  ```
  ros2 run uned_crazyflie_missions formation --ros-args -p config:=/path/to/formation.yaml
  ```

- **`waypoints`** (`waypoints.py`): drives one robot through a sequence of points read from a config file — `take_off`, visit every point (in the declared order, or TSP-optimized, see `shape`), `land`. Matches `uned_crazyflie_config/resources/demo_individual_waypoints.yaml` exactly (see that file for the full schema: `output`/`input` topics and types, `range` tolerance, `shape` (`polygon` or `tsp`), `repeat`, `period`, and a `points` dict with per-point `x`/`y`/`z` and an optional `t` timeout). Ordering and the pure TSP heuristic (`nearest_neighbor` + `two_opt`) still live in `tsp.py`, unchanged.
  ```
  ros2 run uned_crazyflie_missions waypoints --ros-args -p config:=/path/to/demo_individual_waypoints.yaml
  ```

- **`sequencer`** (`sequencer.py`): publishes a scripted sequence of values to arbitrary topics, each step gated by a fixed delay or by waiting for a specific value on a subscribed topic. Matches `uned_crazyflie_config/resources/demo_individual_waypoints_topics.yaml` exactly (declare `publisher`/`subscription` topics + types, then `cmd00`, `cmd01`, ... in order, each with a `topic`/`type`/`value` and a `trigger`). Only `std_msgs/String` is implemented today, since it's the only type used in the reference config — not a silent limitation, `SUPPORTED_TYPES` in the module makes it explicit and it raises a clear error otherwise.
  ```
  ros2 run uned_crazyflie_missions sequencer --ros-args -p config:=/path/to/demo_individual_waypoints_topics.yaml
  ```

## Adding a new mission

Same pattern as the three nodes above: a node that takes a single `config` parameter (path to a `.yaml`), talks only over the topics named in that config, and never imports from `uned_crazyflie_driver` or `cflib`. Register it as a `console_script` in `setup.py`.

## Tests

Beyond the standard lint tests (`ament_copyright`, `ament_flake8`, `ament_pep257`), every node's pure logic is unit-tested without a running ROS graph:

- **`test_tsp.py`** (4 tests): `nearest_neighbor` visits every point once, `two_opt` never worsens a tour, `solve_tsp` finds the true optimum on a 4-point square (perimeter, not a diagonal crossing), and the 2-point edge case.
- **`test_waypoints.py`** (4 tests): `load_points` preserves yaml declaration order and defaults a missing `t` to 0; `order_for_shape('polygon', ...)` is the declared order; `order_for_shape('tsp', ...)` finds the real optimum (same square as above, checked with an actual path-length comparison, not just "some permutation"); a single-point edge case.
- **`test_formation.py`** (4 tests): `compute_correction` is zero with no neighbours, zero when already at the desired offset, pulls toward the desired offset when too far, and is the *mean* of several neighbours' errors (checked with values chosen so the mean has to actually cancel, not just "any non-crashing number").
- **`test_sequencer.py`** (4 tests): `sorted_cmd_keys` orders `cmd00`/`cmd01`/... correctly (including two-digit numbers) and ignores unrelated keys; `trigger_satisfied` for both `time` and `topic` triggers, plus an unknown trigger type.

**Deliberately not unit-tested**: the `Node` subclasses themselves (`FormationNode`, `WaypointsNode`, `SequencerNode`) — wiring a meaningful test for them needs a running ROS graph, which is out of scope for a unit test here. `waypoints.py`'s state machine was checked with a live `ros2 run` integration test against a harness node during the original TSP-only version's development (see `AUDIT.md` on the `doc` branch); that harness test needs re-running against the new config-driven interface, not done in this pass.
