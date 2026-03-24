// Copyright 2024 viswa
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_PROXIMITY_WAIT__CHECK_ROBOT_PROXIMITY_HPP_
#define NAV2_PROXIMITY_WAIT__CHECK_ROBOT_PROXIMITY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_proximity_wait
{

/**
 * @brief A BehaviorTree action node that checks if another robot is
 *        within a safety radius. Returns RUNNING (blocking the BT tree)
 *        when a robot is too close, and SUCCESS otherwise.
 *
 *        Implements hysteresis: once paused (RUNNING), the robot only resumes
 *        (SUCCESS) when the distance exceeds clear_radius (> safety_radius).
 *
 *        Priority: the robot with the lower numeric ID (e.g. agent1 < agent2)
 *        has right-of-way and will NOT pause when within safety_radius. The
 *        higher-ID robot yields (returns RUNNING) until the area clears.
 */
class CheckRobotProximity : public BT::ActionNodeBase
{
public:
  /**
   * @brief Constructor
   * @param name The name of the BT node instance
   * @param config The BT node configuration (blackboard access)
   */
  CheckRobotProximity(
    const std::string & name,
    const BT::NodeConfiguration & config);

  /**
   * @brief Default destructor
   */
  ~CheckRobotProximity() = default;

  /**
   * @brief Declares the ports this node uses on the BT blackboard.
   *        Inputs read from bt_navigator parameters:
   *          - other_robot_frames: semicolon-separated list of other robots' base frames
   *          - safety_radius:      distance (m) at which to begin pausing
   *          - clear_radius:       distance (m) at which to resume after pausing
   * @return The set of declared BT ports
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Called every BT tick. Looks up TF distances to all watched frames.
   * @return BT::NodeStatus::SUCCESS  — clear, navigation continues
   *         BT::NodeStatus::RUNNING  — too close (this robot yields), tree blocked
   *
   *         Priority: if self_priority_ < other robot's priority, we have
   *         right-of-way and always return SUCCESS regardless of distance.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Called by the BT engine when the node is halted (e.g. goal cancel).
   *        Resets the paused state so the next nav goal starts fresh.
   */
  void halt() override;

private:
  /**
   * @brief Performs a TF lookup to get the Euclidean distance between two frames.
   * @param frame_a  Source TF frame id
   * @param frame_b  Target TF frame id
   * @param distance Output: computed Euclidean distance in metres
   * @return true if the lookup succeeded, false otherwise
   */
  bool getDistance(
    const std::string & frame_a,
    const std::string & frame_b,
    double & distance);

  /**
   * @brief Extracts the numeric priority from a frame name like "agent2/base_link".
   * @return Integer robot number (e.g. 2), or 0 if none found.
   */
  static int extractPriority(const std::string & frame);

  /// ROS2 node handle obtained from the BT blackboard (shared with bt_navigator)
  rclcpp::Node::SharedPtr node_;

  /// TF2 buffer used for transform lookups
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /// TF2 transform listener (keeps tf_buffer_ populated)
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /// This robot's own base_link frame — derived at runtime from the node namespace,
  /// so the same plugin binary works for any robot regardless of its name.
  std::string self_frame_;

  /// List of other robots' base_link frames to monitor
  std::vector<std::string> other_frames_;

  /// Distance threshold (m) at which the robot begins pausing
  double safety_radius_;

  /// Distance threshold (m) at which a paused robot resumes (must be > safety_radius_)
  double clear_radius_;

  /// Hysteresis state: true if currently paused (RUNNING mode)
  bool is_paused_;

  /// Numeric priority derived from own namespace (e.g. agent1 → 1).
  /// Lower number has right-of-way — it does NOT yield.
  int self_priority_;
};

}  // namespace nav2_proximity_wait

#endif  // NAV2_PROXIMITY_WAIT__CHECK_ROBOT_PROXIMITY_HPP_
