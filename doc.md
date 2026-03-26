# `nav2_proximity_wait` — Technical Documentation

> **Package version:** 0.0.0  
> **Build type:** `ament_cmake`  
> **License:** Apache-2.0 (source headers)  
> **ROS 2 distro target:** Humble  
> **Plugin type:** Nav2 Behavior Tree shared library plugin

---

## 1. Purpose

`nav2_proximity_wait` adds **decentralized, reactive collision avoidance** between multiple Jackal robots in the CHARS swarm without any central coordinator. It does this by providing a single custom Nav2 Behavior Tree node — `CheckRobotProximity` — that is inserted directly into each robot's navigation BT tree.

### The Problem It Solves

In a multi-robot Nav2 deployment, independent robots navigate towards their own goals without awareness of each other. When two robots approach the same area simultaneously (e.g., approaching adjacent pick zones or a shared corridor), their local costmaps may not respond fast enough to prevent a near-collision. Existing Nav2 costmap inflation layers treat peer robots like static obstacles — they can trigger recovery behaviours (spinning, backing up) rather than simply **waiting for the other robot to pass**.

### The Solution

`CheckRobotProximity` is a BT node placed between path computation and path following. On every BT tick it:

1. Reads the TF positions of all watched peer robots.
2. Computes the **2D Euclidean distance** (X-Y plane only, ignoring Z) from this robot's `base_link` to each peer's `base_link` via the global `/tf` tree.
3. If a peer is closer than `safety_radius`: the node returns **`RUNNING`**, which causes the parent `ReactiveSequence` to immediately **halt `FollowPath`** — the robot stops in place.
4. The robot stays stopped until every peer is beyond `clear_radius` (**hysteresis** — preventing stop/go oscillation).
5. Once clear, the node returns **`SUCCESS`** and `FollowPath` resumes normally.

### Priority-Based Right-of-Way (Deadlock Prevention)

If two robots are simultaneously within each other's safety zones, both would yield → **deadlock**. This is prevented by a deterministic priority rule:

> **The robot with the lower numeric ID has right-of-way and never pauses.**  
> `agent1` (priority 1) always proceeds. `agent2` (priority 2) yields. If `agent3` is also present, it yields to both `agent1` and `agent2`.

Priority is extracted automatically from the robot's ROS namespace (e.g., `agent2/base_link` → priority 2). No configuration is needed — the same compiled plugin binary works for every robot in the fleet.

### Role in CHARS

`nav2_proximity_wait` is a **Layer 1 (Execution & Perception)** safety mechanism. It activates whenever any CHARS task causes two robots to navigate near each other — particularly during pick/place operations in shared workspace zones. It runs completely inside `bt_navigator` and requires no changes to the allocator (Layer 2) or planning pipeline (Layer 3).

---

## 2. Package Structure

```
nav2_proximity_wait/
├── include/
│   └── nav2_proximity_wait/
│       └── check_robot_proximity.hpp   ← Class declaration + full Doxygen API docs
├── src/
│   └── check_robot_proximity.cpp       ← BT node implementation + BT_REGISTER_NODES
├── bt_xml/
│   └── navigate_w_proximity_wait.xml   ← Custom Nav2 BT tree with proximity gate
├── config/
│   ├── bt_navigator_proximity.yaml     ← bt_navigator params: plugin list + default radii
│   └── fleet_config.yaml               ← Fleet namespace + radius definitions
├── CMakeLists.txt
└── package.xml
```

---

## 3. Dependencies

| Dependency | Role |
|---|---|
| `rclcpp` | ROS 2 node handle and parameter API |
| `nav2_behavior_tree` | Base class infrastructure for Nav2 BT plugins |
| `behaviortree_cpp_v3` | `BT::ActionNodeBase`, `BT::NodeStatus`, BT port system |
| `tf2` | Core TF2 data types and `TimePointZero` |
| `tf2_ros` | `tf2_ros::Buffer`, `tf2_ros::TransformListener` |
| `tf2_geometry_msgs` | `geometry_msgs::msg::TransformStamped` conversion |
| `geometry_msgs` | Transform message types |
| `nav2_msgs` | (Transitive; required by nav2_behavior_tree) |
| `nav2_util` | (Transitive; required by nav2_behavior_tree) |

---

## 4. The BT Plugin: `CheckRobotProximity`

### 4.1 Overview

| Property | Value |
|---|---|
| **C++ Class** | `nav2_proximity_wait::CheckRobotProximity` |
| **Base class** | `BT::ActionNodeBase` |
| **BT XML tag** | `<CheckRobotProximity/>` |
| **Shared library** | `nav2_proximity_wait_bt_node` |
| **Registered via** | `BT_REGISTER_NODES` macro (loaded by `bt_navigator` via `dlopen`) |

### 4.2 Return Values

| Return | Meaning | Effect on BT Tree |
|---|---|---|
| `SUCCESS` | All peers are beyond the threshold — safe to navigate | Parent `ReactiveSequence` proceeds to tick `FollowPath` |
| `RUNNING` | A peer robot is too close — this robot is yielding | Parent `ReactiveSequence` immediately **halts** `FollowPath` (robot stops); BT engine re-ticks this node next cycle without spin-looping |

> **Why `RUNNING` instead of `FAILURE`?**  
> A `FAILURE` response from a child inside a `ReactiveSequence` would propagate failure up the tree and trigger costly recovery behaviours (spinning, backing up). `RUNNING` correctly signals "I'm still working on it — wait" to the BT engine, yielding control without triggering recovery.

### 4.3 Tick Logic (State Machine)

```
Each BT tick:

  for each frame in other_robot_frames:
    dist = TF_distance(self_frame, other_frame)   ← 2D Euclidean (X-Y only)

    if TF lookup failed:
      skip (assume no conflict — avoid deadlocking all robots)

    if NOT is_paused:
      if dist < safety_radius:
        if self_priority < other_priority:   ← we have right-of-way
          skip this peer (continue navigating)
        else:
          is_paused = true
          return RUNNING   ← stops FollowPath

    if is_paused:
      if dist < clear_radius:
        return RUNNING   ← hysteresis: stay stopped until fully clear
      # else: this peer is clear, check remaining peers

  # All peers checked and clear:
  if is_paused: is_paused = false  (log RESUMING)
  return SUCCESS
```

### 4.4 Priority Resolution

Priority is extracted by `extractPriority(frame)` which parses the digit sequence from the namespace prefix:

| Frame | Namespace prefix | Priority |
|---|---|---|
| `agent1/base_link` | `agent1` | **1** (highest right-of-way) |
| `agent2/base_link` | `agent2` | **2** |
| `agent3/base_link` | `agent3` | **3** (lowest priority) |
| `base_link` (no namespace) | — | **0** (no priority comparison) |

Rules:
- **Lower number = higher priority = right-of-way.**
- `agent1` never yields to any peer.
- `agent2` yields to `agent1` but not to `agent3`.
- If priority cannot be parsed (0 for either robot), the check falls through and no right-of-way override is applied.

### 4.5 Self-Frame Derivation

The node's own `base_link` frame is derived at runtime from the ROS node namespace — **not** hardcoded:

```cpp
ns = node_->get_namespace();   // e.g. "/agent2"
self_frame_ = ns + "/base_link";  // → "agent2/base_link"
```

This means the same compiled `.so` plugin can be used for `agent1`, `agent2`, `agent3`, etc., without recompilation or separate config.

### 4.6 TF Distance Computation

- Uses `tf_buffer_->lookupTransform(frame_a, frame_b, tf2::TimePointZero)` — always uses the latest available transform.
- Distance is **2D Euclidean** (ground plane only):  
  `dist = sqrt(tx² + ty²)` — Z component ignored (flat floor assumption).
- On TF lookup failure: **throttled warning** (max 1 per 2000 ms) and the peer is skipped — avoids system-wide deadlock if one robot's TF temporarily drops out.

### 4.7 Hysteresis

| Threshold | Purpose |
|---|---|
| `safety_radius` (e.g. 1.0 m) | Triggers pause — robot stops when a peer enters this radius |
| `clear_radius` (e.g. 1.5 m) | Clears pause — robot only resumes when every peer exits this larger radius |

The `clear_radius > safety_radius` gap creates a **dead-band** (e.g. 0.5 m) that prevents rapid stop/start cycling when a peer is hovering near the boundary.

### 4.8 `halt()` Behaviour

When the Nav2 action is cancelled or the BT tree is reset, `halt()` is called:
- Sets `is_paused_ = false` — fresh start for the next goal.
- Sets BT node status to `IDLE`.

This prevents a robot that was mid-pause from being stuck in the paused state when assigned a new task.

---

## 5. BT Ports (Parameters)

The BT ports are **declared for XML compatibility** but the plugin reads its actual values from **ROS node parameters** (set by `bt_navigator` from the YAML config), not from the BT blackboard.

| Port Name | Direction | Type | Default | Description |
|---|---|---|---|---|
| `other_robot_frames` | Input | `string` | `""` | Semicolon-delimited TF frames of peer robots (e.g. `"agent2/base_link;agent3/base_link"`). **Unused — read from ROS param.** |
| `safety_radius` | Input | `double` | `1.0` | Pause threshold in metres. **Unused — read from ROS param.** |
| `clear_radius` | Input | `double` | `1.4` | Resume threshold in metres. **Unused — read from ROS param.** |

---

## 6. ROS Parameters

These parameters must be set on the `bt_navigator` node (under its namespace). They are the **actual** runtime values read by `CheckRobotProximity`.

| Parameter | Type | Example Value | Description |
|---|---|---|---|
| `other_robot_frames` | `string` | `"agent1/base_link;agent3/base_link"` | Semicolon-delimited list of peer robot `base_link` TF frames to monitor. For `agent2`, this would list `agent1/base_link` and any other swarm members. |
| `safety_radius` | `double` | `1.0` | Distance in metres at which this robot begins yielding. Should be ≥ robot footprint diameter + safety margin. |
| `clear_radius` | `double` | `1.5` | Distance in metres at which this robot resumes navigating after a yield. **Must be strictly greater than `safety_radius`** to provide hysteresis. |

These are declared with safe defaults in the constructor if not already set, so the plugin will not crash if parameters are missing — but it will log a warning and no proximity checking will occur if `other_robot_frames` is empty.

---

## 7. Topics / TF Consumed

`CheckRobotProximity` does **not** subscribe to any ROS topics directly. All inter-robot distance information is obtained through the **TF2 transform tree**.

| TF Frames Read | Source | Description |
|---|---|---|
| `<self_ns>/base_link` | Robot's `robot_state_publisher` | This robot's own position in the global map frame |
| `<peer_ns>/base_link` (one per peer) | Each peer's `robot_state_publisher` | Peer robot positions published to the shared `/tf` topic |

All robots must share a common `/tf` topic (the default in a single ROS 2 DDS domain). No remapping is needed.

---

## 8. The Custom BT Tree: `navigate_w_proximity_wait.xml`

This replaces the standard `navigate_to_pose_w_replanning_and_recovery.xml` BT tree that Nav2 uses by default.

### Structure

```
RecoveryNode (retries=6)
├── PipelineSequence "NavigateWithReplanning"
│   ├── RateController (1 Hz)           ← Replan global path every second
│   │   └── RecoveryNode (retries=1) "ComputePathToPose"
│   │       ├── ComputePathToPose
│   │       └── ClearEntireCostmap (global)
│   │
│   └── ReactiveSequence "ProximityGatedFollow"   ← KEY ADDITION
│       ├── CheckRobotProximity          ← ticked FIRST every cycle
│       │     SUCCESS → continue to FollowPath
│       │     RUNNING → halt FollowPath immediately (robot stops)
│       │
│       └── RecoveryNode (retries=1) "FollowPath"
│           ├── FollowPath               ← only runs while proximity check passes
│           └── ClearEntireCostmap (local)
│
└── ReactiveFallback "RecoveryFallback"  ← unchanged standard recovery
    ├── GoalUpdated
    └── RoundRobin "RecoveryActions"
        ├── Sequence "ClearingActions"
        │   ├── ClearEntireCostmap (local)
        │   └── ClearEntireCostmap (global)
        ├── Spin (1.57 rad)
        ├── Wait (5 s)
        └── BackUp (0.30 m, 0.05 m/s)
```

### Key Design Decision: `ReactiveSequence`

`ReactiveSequence` re-evaluates **all children from left to right** on every tick, even if `FollowPath` is already running. This means:

- `CheckRobotProximity` is evaluated before every `FollowPath` tick.
- If `CheckRobotProximity` returns `RUNNING`, the `ReactiveSequence` **immediately calls `halt()` on `FollowPath`** — the robot brakes to a stop within the controller's deceleration limits.
- No manual `Wait` node is needed — `CheckRobotProximity` self-maintains the `RUNNING` state via hysteresis until the area clears.

---

## 9. Configuration Files

### `config/fleet_config.yaml`

The **single source of truth** for the fleet. This YAML is intended to be loaded by the launch file and used to auto-generate per-robot `bt_navigator` parameter blocks.

```yaml
fleet_namespaces:
  - agent1
  - agent2

frame_suffix: "base_link"   # → agent1/base_link, agent2/base_link

safety_radius: 1.0   # metres
clear_radius:  1.5   # metres  (must be > safety_radius)
```

**How to add a robot to the fleet:**  
Add its namespace to `fleet_namespaces` and rebuild `other_robot_frames` in the launch file. The plugin binary requires no recompilation — it auto-detects its own identity from the ROS namespace.

---

### `config/bt_navigator_proximity.yaml`

Provides the complete `bt_navigator` ROS 2 parameter block. Key additions vs. stock Nav2:

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: ""   # Set dynamically by launch file to this package's XML
    other_robot_frames: ""           # Set per-robot by launch file
    safety_radius: 0.0              # Set from fleet_config.yaml by launch file
    clear_radius:  0.0              # Set from fleet_config.yaml by launch file

    plugin_lib_names:
      # ... all standard Nav2 BT plugins ...
      - nav2_proximity_wait_bt_node  # ← added at the end
```

The `other_robot_frames`, `safety_radius`, and `clear_radius` fields are set to neutral defaults in this file and **overridden at launch time** from the launch file (which reads `fleet_config.yaml`).

---

## 10. Building the Package

```bash
cd ~/swarm_ws

# Build (and its Nav2 dependencies)
colcon build --packages-select nav2_proximity_wait

# Source the install overlay
source install/setup.bash
```

The build produces a single shared library:

```
install/nav2_proximity_wait/lib/libnav2_proximity_wait_bt_node.so
```

This `.so` is loaded at runtime by `bt_navigator` via `dlopen` when the plugin name is listed in `plugin_lib_names`.

---

## 11. Integration: Connecting to Nav2

No standalone launch file is provided — this package **integrates into your existing Nav2 launch** (e.g., `swarm_bringup`). The following steps show what must be configured.

### Step 1 — Point `bt_navigator` at the custom BT XML

In your Nav2 launch file or merged params YAML, for each robot:

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: "<path_to>/nav2_proximity_wait/bt_xml/navigate_w_proximity_wait.xml"
```

Or dynamically in Python launch:

```python
from ament_index_python.packages import get_package_share_directory
import os

bt_xml_path = os.path.join(
    get_package_share_directory('nav2_proximity_wait'),
    'bt_xml',
    'navigate_w_proximity_wait.xml'
)
```

### Step 2 — Register the plugin library

Ensure `bt_navigator_proximity.yaml` (or an equivalent block in your Nav2 YAML) is loaded. The critical line is:

```yaml
plugin_lib_names:
  - nav2_proximity_wait_bt_node   # ← must be present
```

### Step 3 — Set per-robot proximity parameters

For each robot, construct `other_robot_frames` as a semicolon-delimited string of all **other** robots' `base_link` frames. Example for a 2-robot fleet:

**agent1's bt_navigator:**
```yaml
bt_navigator:
  ros__parameters:
    other_robot_frames: "agent2/base_link"
    safety_radius: 1.0
    clear_radius: 1.5
```

**agent2's bt_navigator:**
```yaml
bt_navigator:
  ros__parameters:
    other_robot_frames: "agent1/base_link"
    safety_radius: 1.0
    clear_radius: 1.5
```

### Step 4 — Pattern for fleet expansion

Use `fleet_config.yaml` in your launch file to auto-generate these strings:

```python
import yaml, os
from ament_index_python.packages import get_package_share_directory

fleet_cfg = yaml.safe_load(open(os.path.join(
    get_package_share_directory('nav2_proximity_wait'),
    'config', 'fleet_config.yaml'
)))

namespaces = fleet_cfg['fleet_namespaces']   # ['agent1', 'agent2']
suffix     = fleet_cfg['frame_suffix']        # 'base_link'
safety_r   = fleet_cfg['safety_radius']
clear_r    = fleet_cfg['clear_radius']

def other_frames(self_ns):
    """Build semicolon-separated list of all peers (excluding self)."""
    return ';'.join(
        f"{ns}/{suffix}" for ns in namespaces if ns != self_ns
    )

# For agent1: other_frames('agent1') → "agent2/base_link"
# For agent2: other_frames('agent2') → "agent1/base_link"
```

---

## 12. Verifying the Plugin is Loaded

After launching Nav2 with the plugin configured, check the `bt_navigator` log:

```bash
ros2 node list | grep bt_navigator
# e.g.: /agent1/bt_navigator

# Check node logs for plugin registration
ros2 run nav2_util lifecycle_bringup /agent1/bt_navigator
```

Or watch for `[CheckRobotProximity] Initialised.` in the bt_navigator output when the first navigation goal is sent.

Verify TF frames are available:

```bash
ros2 run tf2_tools tf2_echo agent1/base_link agent2/base_link
# Should print a live transform — if it fails, TF is not shared between robots
```

Trigger proximity behaviour (with two robots running):

```bash
# Drive agent2 near agent1 manually
# In bt_navigator logs for agent2, expect:
# [CheckRobotProximity] PAUSING — agent1/base_link is 0.85 m away (safety_radius=1.00 m)
# [CheckRobotProximity] RESUMING — all robots beyond clear_radius (1.50 m)
```

---

## 13. Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| `[CheckRobotProximity] other_robot_frames is empty` | `other_robot_frames` param not set on bt_navigator | Verify your NAv2 launch is merging the proximity YAML and setting the parameter per-robot |
| Plugin not found — bt_navigator crash on launch | `nav2_proximity_wait_bt_node` not in `plugin_lib_names` or `.so` not on `LD_LIBRARY_PATH` | Rebuild the package; source the install overlay; confirm the library is listed in `bt_navigator_proximity.yaml` |
| TF lookup warnings — `agent2/base_link` not found | TF frames not shared between robots | Ensure all robots share the same DDS domain; check that `robot_state_publisher` for each robot is publishing to the global `/tf` topic (not a namespaced one) |
| Both robots pause — deadlock | Priority extraction failed (e.g. non-numeric namespace suffix) | Ensure robot namespaces contain a digit (e.g. `agent1`, `agent2`); avoid `robot_a`, `robot_b` style names |
| Robot never resumes after proxmity event | `clear_radius` ≤ `safety_radius` | Ensure `clear_radius > safety_radius` in `fleet_config.yaml` |
| Robot stops every tick even when no peer nearby | Stale `is_paused_` state after a goal cancel | Ensure the BT tree's `halt()` mechanism reaches `CheckRobotProximity`; verify `halt()` is being called on goal cancel |
| Proximity check triggers recovery spin/backup | Using an older BT tree with `FAILURE` instead of `RUNNING` | Use the provided `navigate_w_proximity_wait.xml` which uses `ReactiveSequence`; do not use `ReactiveFallback` pattern with `FAILURE` |

---

## 14. Integration Notes for CHARS

- This plugin operates entirely inside each robot's `bt_navigator` process — it requires **no changes to the Allocator (Layer 2)** or planner (Layer 3). The allocator is unaware of proximity pauses; from its perspective, the action goal is still `STATUS_EXECUTING` — it will eventually complete or timeout as configured.
- The `safety_radius` should be tuned to be larger than the robot's footprint plus the costmap inflation radius to ensure the proximity gate triggers before the Nav2 costmap-based collision avoidance kicks in.
- In CHARS, `clear_radius` is set to **1.5 m** and `safety_radius` to **1.0 m**, giving a 0.5 m hysteresis band. This was chosen based on the Jackal's ~0.5 m width and typical corridor widths in the Gazebo test world.
- Because the priority scheme uses **namespace digit ordering**, it is critical that CHARS robot namespaces follow the `agentN` convention. If deploying with different naming, update `extractPriority()` or define an explicit priority mapping.
- The 2D distance (X-Y only) is appropriate for flat-floor Gazebo Fortress simulation. If vertical separation is ever relevant (e.g., multi-floor environments), change the distance formula to 3D: `sqrt(tx² + ty² + tz²)`.
