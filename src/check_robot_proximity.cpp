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

#include "nav2_proximity_wait/check_robot_proximity.hpp"

#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_proximity_wait
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
CheckRobotProximity::CheckRobotProximity(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  safety_radius_(1.0),
  clear_radius_(1.4),
  is_paused_(false),
  self_priority_(0)
{
  // Obtain the ROS2 node handle shared by all BT nodes via the blackboard
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Initialise TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Derive self_frame_ from the node namespace so that the same plugin binary
  // works for any robot (agent1, agent2, …) without any hardcoding.
  std::string ns = node_->get_namespace();
  if (ns.empty() || ns == "/") {
    self_frame_ = "base_link";
  } else {
    // Strip leading '/'
    if (ns[0] == '/') {
      ns = ns.substr(1);
    }
    self_frame_ = ns + "/base_link";
  }

  // Derive numeric priority from self_frame_ (e.g. "agent1/base_link" → 1).
  // Lower number = higher priority (right-of-way). This prevents deadlock when
  // both robots are within the safety zone of each other: the lower-ID robot
  // always proceeds, the higher-ID robot always yields.
  self_priority_ = extractPriority(self_frame_);

  // -------------------------------------------------------------------------
  // Read safety_radius, clear_radius, and other_robot_frames from ROS node
  // parameters.  bt_navigator sets these on the node from the params YAML file
  // (written by mobman_nav2.launch.py), so they are reliably available here.
  // -------------------------------------------------------------------------

  // Declare parameters with safe defaults (bt_navigator will have set them)
  if (!node_->has_parameter("safety_radius")) {
    node_->declare_parameter("safety_radius", 1.0);
  }
  if (!node_->has_parameter("clear_radius")) {
    node_->declare_parameter("clear_radius", 1.4);
  }
  if (!node_->has_parameter("other_robot_frames")) {
    node_->declare_parameter("other_robot_frames", std::string(""));
  }

  safety_radius_ = node_->get_parameter("safety_radius").as_double();
  clear_radius_  = node_->get_parameter("clear_radius").as_double();

  // Parse semicolon-delimited other_robot_frames string into a vector
  std::string frames_str = node_->get_parameter("other_robot_frames").as_string();
  if (!frames_str.empty()) {
    std::istringstream ss(frames_str);
    std::string token;
    while (std::getline(ss, token, ';')) {
      if (!token.empty()) {
        other_frames_.push_back(token);
      }
    }
  }

  // Log startup config so it is visible and verifiable in the bt_navigator log
  RCLCPP_INFO(
    node_->get_logger(),
    "[CheckRobotProximity] Initialised."
    "\n  self_frame      : %s"
    "\n  self_priority   : %d  (lower = right-of-way)"
    "\n  safety_radius   : %.2f m"
    "\n  clear_radius    : %.2f m"
    "\n  watched frames  : %zu",
    self_frame_.c_str(),
    self_priority_,
    safety_radius_,
    clear_radius_,
    other_frames_.size());

  for (const auto & f : other_frames_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[CheckRobotProximity]   -> %s  (priority %d)",
      f.c_str(), extractPriority(f));
  }

  if (other_frames_.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[CheckRobotProximity] other_robot_frames is empty — "
      "no proximity checking will occur. "
      "Check that 'other_robot_frames' is set in the bt_navigator ROS params.");
  }
}

// ---------------------------------------------------------------------------
// providedPorts — kept for XML compatibility; values read from ROS params.
// ---------------------------------------------------------------------------
BT::PortsList CheckRobotProximity::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "other_robot_frames",
      "",
      "Semicolon-delimited list of other robots' base_link TF frames (unused: read from ROS param)"),
    BT::InputPort<double>(
      "safety_radius",
      1.0,
      "Distance in metres at which to pause (unused: read from ROS param)"),
    BT::InputPort<double>(
      "clear_radius",
      1.4,
      "Distance in metres at which to resume after a pause (unused: read from ROS param)"),
  };
}

// ---------------------------------------------------------------------------
// tick — main BT logic, called every BT cycle
//
// Returns:
//   SUCCESS  — no robot is too close (or this robot has right-of-way)
//   RUNNING  — a higher-priority robot is too close; this robot is pausing
//
// Returning RUNNING correctly yields control back to the BT scheduler,
// preventing the 100 Hz spin-loop that occurred when FAILURE was returned.
// ---------------------------------------------------------------------------
BT::NodeStatus CheckRobotProximity::tick()
{
  for (const auto & other_frame : other_frames_) {
    double dist = 0.0;
    if (!getDistance(self_frame_, other_frame, dist)) {
      // TF lookup failed — assume no conflict to avoid deadlocking all robots
      continue;
    }

    if (!is_paused_) {
      // Not currently paused — check if we should start pausing
      if (dist < safety_radius_) {
        // Priority check: if we have right-of-way (lower ID), do NOT pause.
        // The other robot (higher ID) will be the one to yield.
        int other_priority = extractPriority(other_frame);
        if (self_priority_ > 0 && other_priority > 0 && self_priority_ < other_priority) {
          // We have right-of-way — skip this robot and keep going
          RCLCPP_DEBUG(
            node_->get_logger(),
            "[CheckRobotProximity] RIGHT-OF-WAY — %s is %.2f m away but we (priority %d) "
            "outrank it (priority %d). Continuing.",
            other_frame.c_str(), dist, self_priority_, other_priority);
          continue;
        }

        is_paused_ = true;
        RCLCPP_WARN(
          node_->get_logger(),
          "[CheckRobotProximity] PAUSING — %s is %.2f m away (safety_radius=%.2f m)",
          other_frame.c_str(), dist, safety_radius_);
        return BT::NodeStatus::RUNNING;  // ← RUNNING, not FAILURE — avoids BT spin-loop
      }
    } else {
      // Currently paused — only clear when distance exceeds clear_radius (hysteresis)
      if (dist < clear_radius_) {
        return BT::NodeStatus::RUNNING;  // still too close, keep yielding
      }
    }
  }

  // All robots checked — none are within the relevant threshold
  if (is_paused_) {
    is_paused_ = false;
    RCLCPP_INFO(
      node_->get_logger(),
      "[CheckRobotProximity] RESUMING — all robots beyond clear_radius (%.2f m)",
      clear_radius_);
  }
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// halt — called by the BT engine when the node is interrupted (e.g. goal
// cancelled, tree reset).  Reset paused state so the next nav goal starts
// fresh and does not inherit stale state from the previous execution.
// ---------------------------------------------------------------------------
void CheckRobotProximity::halt()
{
  is_paused_ = false;
  setStatus(BT::NodeStatus::IDLE);
}

// ---------------------------------------------------------------------------
// extractPriority — parses the numeric ID out of a frame like "agent2/base_link"
// ---------------------------------------------------------------------------
int CheckRobotProximity::extractPriority(const std::string & frame)
{
  // Grab the portion before the first '/' (e.g. "agent2")
  std::string prefix = frame.substr(0, frame.find('/'));
  std::string digits;
  for (char c : prefix) {
    if (std::isdigit(static_cast<unsigned char>(c))) {
      digits += c;
    }
  }
  if (digits.empty()) {
    return 0;
  }
  try {
    return std::stoi(digits);
  } catch (...) {
    return 0;
  }
}

// ---------------------------------------------------------------------------
// getDistance — TF lookup helper
// ---------------------------------------------------------------------------
bool CheckRobotProximity::getDistance(
  const std::string & frame_a,
  const std::string & frame_b,
  double & distance)
{
  try {
    geometry_msgs::msg::TransformStamped t =
      tf_buffer_->lookupTransform(frame_a, frame_b, tf2::TimePointZero);

    const auto & tr = t.transform.translation;
    // Use 2D Euclidean distance (ignore Z) for ground robots
    distance = std::sqrt(tr.x * tr.x + tr.y * tr.y);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,  // ms — throttle to avoid log spam
      "[CheckRobotProximity] TF lookup failed (%s -> %s): %s",
      frame_a.c_str(), frame_b.c_str(), ex.what());
    return false;
  }
}

}  // namespace nav2_proximity_wait

// ---------------------------------------------------------------------------
// Plugin registration — name must match the XML tag in the BT XML file
// ---------------------------------------------------------------------------
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_proximity_wait::CheckRobotProximity>("CheckRobotProximity");
}
