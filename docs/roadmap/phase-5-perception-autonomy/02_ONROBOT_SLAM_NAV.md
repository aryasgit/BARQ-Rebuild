# P5-02 — SLAM and nav2 on the real robot

> Phase P5 · verified against repo @ 0e5ddaf

## Objective
Take the sim-proven autonomy stack (lidar → slam_toolbox → nav2 RPP → trot gait) onto the real
robot: compose a `robot.launch.py`, map one room by teleop to a measurable quality bar, then
run autonomous A→B missions under the hardware mission protocol distilled from the sim
compute lessons. End state: G5.6 — the robot replans around a human who steps into its path.

## Prerequisites
- P4 gates passed: untethered walking under `/cmd_vel`, estimator running (sim-measured drift
  4–5 % of distance, research log §2e — hardware drift gets its own measurement here).
- P5-01 gates G5.1–G5.3 passed (clean, level, filtered `/scan` on the standing robot).
- One room ≥ 3 × 3 m with wall features (an EMPTY featureless space maps nothing — sim lesson,
  research log §2b), obstacles removable, a spotter.
- Mac on the same LAN for SSH; VNC and/or Foxglove viewing path working (memory:
  barq-remote-viz). Tape measure.
- Mission rule from day one: **no GUI renderers on the Jetson during gated runs** (§5).

---

## 1. Bringup composition — `barq_bringup/launch/robot.launch.py`

`real.launch.py` is the hardware control stack ONLY (ros2_control + rsp + spawners + gait).
Spec: a new `robot.launch.py` that includes it and adds perception/autonomy with the same flag
structure as `sim.launch.py` (`slam:=`, `nav:=`, flags default false). Build it from these
pieces:

1. **Include `real.launch.py`** (gait:=true passes through; `device:=/dev/barq_teensy`).
2. **State estimator — MUST be added.** `real.launch.py` today has NO estimator node; on
   hardware the estimator IS odometry (there is no ground truth, no `/odom_gt`, no `odom_tf`
   node). Lift the `state_estimator` Node block from `sim.launch.py` (lines ~159–167), strip
   `use_sim_time`, and set `publish_tf: True` unconditionally. Spec an `odom_source` launch
   arg that only accepts `estimated` on hardware — make `ground_truth` raise, so nobody
   copy-pastes a sim habit (the silent failure would be: no odom→base_link TF, SLAM dead).
3. **Lidar driver node** (P5-01 §3 choice) + optional `scan_to_scan_filter_chain` behind a
   `scan_filter:=` flag (P5-01 §5).
4. **slam_toolbox** — lift the Node block from `sim.launch.py` (lines ~169–176): package
   `slam_toolbox`, executable `async_slam_toolbox_node`, parameters
   `barq_slam_real.yaml` (below), condition `IfCondition(slam)`. Drop the `sim_time` dict.
5. **nav2** — lift the IncludeLaunchDescription from `sim.launch.py` (lines ~89–98):
   `nav2_bringup/launch/navigation_launch.py` with `use_sim_time: 'false'` and
   `params_file: barq_nav2_real.yaml`, condition `IfCondition(nav)`.
6. **No bridges, no spawn, no odom_tf** — those are Gazebo artifacts; nothing else from
   `sim.launch.py` transfers.

All of it runs in ONE `barq:dev` container (one DDS stack — HANDOFF rule). Any second
container (teleop, bag recording) must mount `-v /dev/shm:/dev/shm` and share the
`ROS_DOMAIN_ID`, or stay out of the graph entirely.

### 1.1 Hardware config copies (do not edit the sim yamls in place)
Copy `barq_slam.yaml` → `barq_slam_real.yaml` and `barq_nav2.yaml` → `barq_nav2_real.yaml` in
`barq_bringup/config/`, then apply exactly these deltas:

| File | Key | Sim value | Hardware value | Why |
|---|---|---|---|---|
| barq_slam_real.yaml | `use_sim_time` | true | **false** | no /clock on hardware |
| barq_slam_real.yaml | `scan_topic` | /scan | /scan_filtered (only if filter active) | P5-01 §5 |
| barq_nav2_real.yaml | every section's `use_sim_time` | true | false — handled by passing `use_sim_time:='false'` to navigation_launch.py (it rewrites the yaml); set the file copies to false anyway so the file tells the truth | no /clock |
| barq_nav2_real.yaml | `bt_navigator.odom_topic` | /odom_gt | **/odom_est** | /odom_gt does not exist on hardware |
| barq_nav2_real.yaml | `velocity_smoother.odom_topic` | /odom_gt | **/odom_est** | same |
| barq_nav2_real.yaml | `FollowPath.desired_linear_vel` | 0.22 | **0.10** | hardware start ceiling (§4) |
| barq_nav2_real.yaml | `velocity_smoother.max_velocity` | [0.22, 0.06, 0.5] | **[0.10, 0.06, 0.5]** | cap must match the ceiling |
| barq_nav2_real.yaml | both `obstacle_layer.scan.topic` | /scan | /scan_filtered (only if filter active) | P5-01 §5 |

Everything else in both yamls transfers as-is — RPP cost+curvature regulated scaling, the
patient progress checker (5 cm / 30 s), goal tolerance 0.15 m, recovery behaviors
spin/backup/drive_on_heading/wait (they earned their place: `spin completed successfully`,
`wait completed successfully` self-recoveries in the sim course, research log §2d). Leave
recoveries ON.

### 1.2 TF chain verification (before any mapping)
With `robot.launch.py slam:=true` up and the robot standing:
```bash
timeout -k 2 10 ros2 run tf2_ros tf2_echo odom base_link   # estimator alive (50 Hz)
timeout -k 2 10 ros2 run tf2_ros tf2_echo map odom         # slam_toolbox alive
timeout -k 2 10 ros2 run tf2_ros tf2_echo map laser        # full chain composes
timeout -k 2 30 ros2 topic hz /scan --window 50            # still ≥ 9.5 Hz under load
```
Walk the robot 1 m by teleop and back: odom→base_link must move ~1 m and return near zero
(estimator sanity); map→odom should stay small while SLAM agrees with odometry. If
odom→base_link is missing: the estimator isn't publishing TF — §1 item 2 was skipped or
`publish_tf` is false.

---

## 2. First map — teleop protocol

Teleop path A (spec): `teleop_twist_keyboard` run INSIDE the robot's container over SSH from
the Mac. Keyboard events ride the SSH channel; DDS never crosses the wifi. Note on
ROS_DOMAIN_ID: `docker exec` into the running container inherits the right one; a separate
container must match it (and mount /dev/shm) or the keys will type into the void.
```bash
ssh barq@barq.local
docker exec -it <robot container> bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.08 -p turn:=0.3
# verify the params took (the banner prints current speed/turn); if this build has no such
# params, press z / x until the printed speed is <= 0.10 m/s
```
Teleop path B: joystick — TBD (hardware not chosen; fill via a bench session with `joy` +
`teleop_twist_joy` if bought; record in the log).

Safety check FIRST, robot on the floor, spotter ready: hold a walk key — robot walks while
held (terminal autorepeat outruns the 1 s `cmd_vel` deadman in the gait planner); release —
robot must stop within ~1 s. If it keeps walking, the deadman regressed: stop, fix, re-test
(research log §2c — the deadman exists because live scrutiny demanded it).

Mapping run:
1. Start `robot.launch.py gait:=true slam:=true` (no nav yet). Verify §1.2.
2. Drive ONE slow loop of the room hugging ~0.5 m from walls, pausing 2 s at corners
   (slam_toolbox integrates across gait cycles — `minimum_travel_distance: 0.2` /
   `minimum_travel_heading: 0.3` mean the map only updates on real displacement; creeping is
   fine, spinning in place is not).
3. Return to the exact start tile and stop. Watch `/map` on the Mac (Foxglove, or RViz-over-
   VNC — allowed here, this is a setup session, not a gated mission).

### 2.1 Map quality bar (gate G5.4)
- Walls straight by eye (no banana rooms), single-line — **no double walls** anywhere (a double
  wall = loop closure failed or odometry drifted beyond what SLAM absorbed).
- Loop closure on return: the map around the start tile stays consistent when you arrive back
  (no visible shear when the loop closes).
- **Tape check**: measure two wall-to-wall distances of the real room (long axis, short axis).
  Compare against the map (count cells × 0.05 m in a saved map image, or Foxglove measure
  tool). Bar: both within ±5 %.
- Save artifacts (§2.2) and a screenshot for the log.

Fail → fallback ladder §6 (SLAM rung).

### 2.2 Map save + reuse
```bash
mkdir -p ~/maps
timeout -k 2 30 ros2 run nav2_map_server map_saver_cli -f ~/maps/room1
timeout -k 2 15 ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/maps/room1_graph'}"
```
(`map_saver_cli` gives the portable .pgm/.yaml; serialize gives slam_toolbox's own pose-graph
for localization mode. Paths must be container-visible — bind-mount ~/maps.)

Reuse — localization mode, yaml deltas on a further copy `barq_slam_localize.yaml`:

| Key | Mapping value | Localization value |
|---|---|---|
| `mode` | mapping | **localization** |
| `map_file_name` | (absent) | `/home/barq/maps/room1_graph` (no extension) |
| `map_start_pose` | (absent) | `[0.0, 0.0, 0.0]` — start the robot at the mapped origin tile, same heading |

P5 missions may simply stay in mapping mode (the sim did; the map keeps improving).
Localization mode is for reusing a finished map across sessions and is the cheaper-CPU option
(see P5-03 ladder C).

---

## 3. nav2 on hardware — first missions

1. Launch the full stack: `robot.launch.py gait:=true slam:=true nav:=true scan_filter:=true`.
   No other GUI/renderer processes on the Jetson — check with `top` (P5-03 baseline (c) is
   measured in exactly this state).
2. **Footprint reality check** (once, before the first goal): stand the robot on paper, mark
   the body extremes and all four feet at stance, measure the max half-diagonal from body
   center. `barq_nav2_real.yaml` assumes `robot_radius: 0.18` (both costmaps) + 
   `inflation_radius: 0.30`. If the measured half-diagonal > 0.18, raise `robot_radius` in
   BOTH costmaps to the measured value + 1 cm and re-check doorway clearance (a 1 m doorway
   minus 2 × (radius + inflation) is the planner's corridor — at 0.18/0.30 there is just
   0.04 m of slack; a bigger radius may need `inflation_radius` trimmed toward 0.25).
3. First goal: 2 m, open floor, no doorway. Send it with the mission runner (§5), spotter
   ready. Then extend to the G5.5 course.

### Speed ceiling policy
Start at `desired_linear_vel: 0.10` / smoother `[0.10, 0.06, 0.5]` (§1.1). Only after G5.5
passes, raise in steps 0.10 → 0.15 → 0.22 (the sim-proven value), re-running the G5.5 course
once per step. The cost+curvature regulation does the slowing near obstacles — the ceiling is
what you are buying confidence in. Keys to touch per step: the two in §1.1, nothing else.

---

## 4. Mission protocol (sim lessons, binding)

From research log §2d + HANDOFF: full load on the Orin silently lost nav2 action goals; exit
codes lied; the CLI swallowed SIGTERM. Therefore, on hardware:

1. **No GUI renderers on the Jetson during missions.** RViz-over-VNC renders ON the Jetson —
   setup sessions only. Mission-time viewing = Foxglove on the Mac via `foxglove_bridge`
   (candidate pkg `ros-humble-foxglove-bridge` — verify installable), subscribed selectively,
   or nothing live + bag review.
2. **Robust Python action client, never fire-and-forget CLI.** The `_tour.py` pattern from the
   sim course sessions (send → verify acceptance → poll result with timeout). The script was
   session-scratch and is not in the repo — the skeleton below IS the spec; commit it as
   `barq_bringup/scripts/mission_runner.py`:
   ```python
   #!/usr/bin/env python3
   """Mission runner — robust action client (sim lesson: CLI goals get lost under load)."""
   import sys, time
   import rclpy
   from rclpy.action import ActionClient
   from rclpy.node import Node
   from nav2_msgs.action import NavigateToPose

   x, y = float(sys.argv[1]), float(sys.argv[2])
   ACCEPT_S, RESULT_S = 10.0, 600.0

   rclpy.init()
   node = Node('mission_runner')
   client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
   if not client.wait_for_server(timeout_sec=ACCEPT_S):
       sys.exit('FAIL: navigate_to_pose server not up')
   goal = NavigateToPose.Goal()
   goal.pose.header.frame_id = 'map'
   goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
   goal.pose.pose.orientation.w = 1.0
   fut = client.send_goal_async(goal)
   rclpy.spin_until_future_complete(node, fut, timeout_sec=ACCEPT_S)
   handle = fut.result()
   if handle is None or not handle.accepted:
       sys.exit('FAIL: goal NOT ACCEPTED — the silent-loss symptom; see P5-03 (load)')
   node.get_logger().info('goal accepted')
   rfut = handle.get_result_async()
   deadline = time.time() + RESULT_S
   while rclpy.ok() and not rfut.done() and time.time() < deadline:
       rclpy.spin_once(node, timeout_sec=1.0)
   status = rfut.result().status if rfut.done() else -1
   print('status:', status, '(4 = SUCCEEDED)')
   sys.exit(0 if status == 4 else 1)
   ```
3. **Verify by telemetry, not exit codes.** The log lines that matter (watch the launch
   console or `ros2 launch` log dir):
   - `bt_navigator`: "Begin navigating from current location to (x, y)" — goal truly entered
     the tree. **Goal accepted but this line absent within ~10 s = the lost-goal symptom.**
   - `controller_server`: "Passing new path to controller" — replanning is happening.
   - `behavior_server`: "Running spin" / "spin completed successfully" / "wait completed
     successfully" — recovery engaged and finished (this exact pair appeared in the sim course
     self-recovery, research log §2d).
   - `bt_navigator`: "Goal succeeded" — corroborate with the runner's status 4 AND a physical
     tape measurement.
4. **`timeout -k 2 <N>` on every ros2 CLI invocation.** No exceptions — the plain CLI wedged
   four sim sessions.
5. One mission at a time; fresh stack restart between gated attempts (between-run state is
   part of the experiment — protocol §2).

---

## 5. Acceptance gates (P5-02)
| Gate | Bar |
|---|---|
| **G5.4** | One-room map passes the quality bar: straight single walls, loop closure at start tile, two tape-measured wall distances within ±5 % |
| **G5.5** | A→B mission, ≥ 5 m path through one ≥ 0.9 m doorway: **3 consecutive successes** (fresh stack each), runner status 4 + "Goal succeeded" in log, final position error vs the tape-marked goal point **< 0.3 m**, zero human touches |
| **G5.6** | Mission with a human stepping into the path mid-run and standing still: robot replans around OR recovers (spin/wait/backup) and completes — without contact. 2 of 3 attempts |

Mission error measurement (no ground truth exists): tape-mark the goal point on the floor
before the run; after "Goal succeeded", tape-measure from the mark to the projection of
base_link center (the deck's center seam) on the floor.

## 6. Fallback ladders
- **SLAM/map (G5.4 fails)**: A — re-verify the chain (§1.2) + scan QA gates still green
  (G5.1–G5.3) + estimator 1 m sanity; most "SLAM is broken" is a dead TF or a wobbling mount.
  B — drive slower with longer corner pauses; raise `minimum_travel_distance` to 0.3 to
  integrate across more gait cycles. C — map the room in two half-loops, serialize after each
  (continue mapping from a serialized graph). Switch rung after 2 failed mapping runs each.
- **Missions (G5.5 fails)**: A — read the BT log against §4.3: goal never entered the tree →
  P5-03 (compute), goal entered but path stalls → B. B — geometry: footprint/inflation check
  (§3.2), doorway slack; widen the course, pass, then narrow back. C — drop ceiling back to
  0.10 if it was raised; re-gate. D — two failures with recoveries firing endlessly: raise
  `xy_goal_tolerance` 0.15 → 0.20 and log it as a hardware delta to revisit.
- **Estimator drift kills map quality** (signature: map shears proportional to distance
  walked, SLAM corrections grow): measure drift over a tape-measured 3 m straight; if ≫ the
  sim's 4–5 %, fix the estimator against hardware reality FIRST (P4 scope) — SLAM cannot
  absorb arbitrary odometry lies. Park P5-02 at G5.4, log it, return.

## 7. Rollback
All changes are additive copies (`*_real.yaml`, `robot.launch.py`, `mission_runner.py`); sim
configs and `real.launch.py` are untouched. `nav:=false slam:=false` returns the robot to a
P4 teleop walker. Maps are files — delete or re-map at will. Speed-ceiling raises roll back by
reverting the two §1.1 keys.

## 8. Artifacts → docs/05_RESEARCH_LOG.md
Map screenshot + .pgm/.yaml + serialized graph · tape-vs-map numbers · estimator drift over
3 m · footprint measurement + any radius change · per-gate mission table (goal, status, final
error, recoveries fired) · every yaml delta actually applied (the §1.1 table, as-built) ·
selective rosbag of one G5.5 success (topic list per P5-03 §4A).

## Escape hatch
If hardware SLAM cannot reach G5.4 after the full ladder (and the estimator passes its drift
check), park P5-02 in docs/04_OPEN_QUESTIONS.md with the maps + bags attached, and run P7
course prep under teleop in the meantime — the robot is still a walking robot. If missions
fail only under compute pressure, the blocker belongs to P5-03, not here: do not tune nav2
parameters to mask a starved CPU (one change at a time, and the right change).
