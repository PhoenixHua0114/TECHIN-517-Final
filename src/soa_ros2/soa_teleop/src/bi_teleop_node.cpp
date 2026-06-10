// Copyright 2024 SOA Teleop
// SPDX-License-Identifier: Apache-2.0

#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class BiTeleopNode : public rclcpp::Node
{
public:
  BiTeleopNode()
  : Node("bi_teleop_node"),
    left_arm_joints_({"left_shoulder_pan", "left_shoulder_lift",
                      "left_elbow_flex", "left_wrist_flex", "left_wrist_roll"}),
    left_gripper_joint_("left_gripper"),
    right_arm_joints_({"right_shoulder_pan", "right_shoulder_lift",
                       "right_elbow_flex", "right_wrist_flex", "right_wrist_roll"}),
    right_gripper_joint_("right_gripper")
  {
    left_arm_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/follower/left_arm_fwd_controller/commands", 10);
    left_gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/follower/left_gripper_fwd_controller/commands", 10);
    right_arm_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/follower/right_arm_fwd_controller/commands", 10);
    right_gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/follower/right_gripper_fwd_controller/commands", 10);

    leader_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/leader/joint_states", 10,
      std::bind(&BiTeleopNode::on_joint_states, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Bi-manual teleop node started — relaying leader → follower (left + right)");
  }

private:
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Build name → position lookup
    std::unordered_map<std::string, double> pos_map;
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      if (i < msg->position.size()) {
        pos_map[msg->name[i]] = msg->position[i];
      }
    }

    // --- Left arm ---
    std_msgs::msg::Float64MultiArray left_arm_cmd;
    left_arm_cmd.data.reserve(left_arm_joints_.size());
    for (const auto & joint : left_arm_joints_) {
      auto it = pos_map.find(joint);
      if (it == pos_map.end()) {
        if (!warned_missing_left_arm_) {
          RCLCPP_WARN(get_logger(),
            "Left arm joint '%s' not found in leader joint states — skipping message",
            joint.c_str());
          warned_missing_left_arm_ = true;
        }
        return;
      }
      left_arm_cmd.data.push_back(it->second);
    }

    // --- Left gripper ---
    auto lgit = pos_map.find(left_gripper_joint_);
    if (lgit == pos_map.end()) {
      if (!warned_missing_left_gripper_) {
        RCLCPP_WARN(get_logger(),
          "Left gripper joint '%s' not found in leader joint states — skipping message",
          left_gripper_joint_.c_str());
        warned_missing_left_gripper_ = true;
      }
      return;
    }

    // --- Right arm ---
    std_msgs::msg::Float64MultiArray right_arm_cmd;
    right_arm_cmd.data.reserve(right_arm_joints_.size());
    for (const auto & joint : right_arm_joints_) {
      auto it = pos_map.find(joint);
      if (it == pos_map.end()) {
        if (!warned_missing_right_arm_) {
          RCLCPP_WARN(get_logger(),
            "Right arm joint '%s' not found in leader joint states — skipping message",
            joint.c_str());
          warned_missing_right_arm_ = true;
        }
        return;
      }
      right_arm_cmd.data.push_back(it->second);
    }

    // --- Right gripper ---
    auto rgit = pos_map.find(right_gripper_joint_);
    if (rgit == pos_map.end()) {
      if (!warned_missing_right_gripper_) {
        RCLCPP_WARN(get_logger(),
          "Right gripper joint '%s' not found in leader joint states — skipping message",
          right_gripper_joint_.c_str());
        warned_missing_right_gripper_ = true;
      }
      return;
    }

    // All joints found — clear any previous warnings and publish
    warned_missing_left_arm_ = false;
    warned_missing_left_gripper_ = false;
    warned_missing_right_arm_ = false;
    warned_missing_right_gripper_ = false;

    left_arm_pub_->publish(left_arm_cmd);

    std_msgs::msg::Float64MultiArray left_gripper_cmd;
    left_gripper_cmd.data.push_back(lgit->second);
    left_gripper_pub_->publish(left_gripper_cmd);

    right_arm_pub_->publish(right_arm_cmd);

    std_msgs::msg::Float64MultiArray right_gripper_cmd;
    right_gripper_cmd.data.push_back(rgit->second);
    right_gripper_pub_->publish(right_gripper_cmd);
  }

  std::vector<std::string> left_arm_joints_;
  std::string left_gripper_joint_;
  std::vector<std::string> right_arm_joints_;
  std::string right_gripper_joint_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_gripper_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr leader_sub_;

  bool warned_missing_left_arm_{false};
  bool warned_missing_left_gripper_{false};
  bool warned_missing_right_arm_{false};
  bool warned_missing_right_gripper_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BiTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
