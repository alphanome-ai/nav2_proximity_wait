# nav2_proximity_wait

Custom Nav2 Behavior Tree plugin for multi-robot proximity-aware yielding in CHARS.

This package adds a BT node, `CheckRobotProximity`, that pauses a robot when a higher-priority robot is too close, then resumes automatically once the area is clear.

## What This Package Does

- Implements `CheckRobotProximity` as a Nav2 BT plugin (`nav2_proximity_wait_bt_node`).
- Uses TF lookups to measure 2D distance between robots.
- Uses priority-based right-of-way from robot namespace IDs (`agent1` > `agent2` in priority).
- Uses hysteresis thresholds:
    - `safety_radius`: start yielding
    - `clear_radius`: resume motion
- Integrates into a custom Nav2 BT XML where `FollowPath` is gated by proximity checks.

When yielding, the node returns `RUNNING` (not `FAILURE`), which cleanly pauses progress without creating tight BT spin loops.

## Package Structure

```text
nav2_proximity_wait/
    include/nav2_proximity_wait/check_robot_proximity.hpp
    src/check_robot_proximity.cpp
    bt_xml/navigate_w_proximity_wait.xml
    config/bt_navigator_proximity.yaml
    config/fleet_config.yaml
    CMakeLists.txt
    package.xml
```

## Dependencies

From `package.xml`:

- `rclcpp`
- `nav2_behavior_tree`
- `behaviortree_cpp_v3`
- `geometry_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `nav2_msgs`
- `nav2_util`

## Build

From your ROS 2 workspace root:

```bash
colcon build --packages-select nav2_proximity_wait
source install/setup.bash
```

## Integration with Nav2

### 1) Register plugin library in bt_navigator params

Use `config/bt_navigator_proximity.yaml` and ensure `plugin_lib_names` includes:

```yaml
- nav2_proximity_wait_bt_node
```

### 2) Use proximity-aware BT XML

Point Nav2 `default_nav_to_pose_bt_xml` to:

```text
<this_package>/bt_xml/navigate_w_proximity_wait.xml
```

### 3) Provide runtime parameters to bt_navigator

The node reads these ROS parameters directly from `bt_navigator`:

- `other_robot_frames` (semicolon-delimited TF frames)
- `safety_radius`
- `clear_radius`

Example values:

```yaml
other_robot_frames: "agent1/base_link;agent3/base_link"
safety_radius: 1.0
clear_radius: 1.5
```

## Fleet Configuration Pattern

`config/fleet_config.yaml` can be used as the single source of truth for swarm namespaces and radii, then expanded in your launch logic into per-robot `bt_navigator` parameters.

## Behavior Summary

On each BT tick:

1. Compute distance from self frame to every frame in `other_robot_frames`.
2. If distance < `safety_radius` and another robot has higher priority, return `RUNNING` (yield).
3. While yielding, stay in `RUNNING` until distance > `clear_radius`.
4. Then return `SUCCESS`, allowing `FollowPath` to continue.

Priority is extracted from frame namespace digits (example: `agent2/base_link` -> priority 2). Lower number means higher right-of-way.

## CHARS Context

This package supports Layer 1 execution robustness in CHARS by reducing local navigation conflicts in multi-robot operation, especially in narrow shared corridors and approach zones.

## Known Metadata Note

`package.xml` currently has placeholder fields for description and license (`TODO`). Update these before publishing the repository to avoid package indexing and compliance confusion.

## License

Source files include Apache-2.0 headers. Align `package.xml` license value accordingly before release.
