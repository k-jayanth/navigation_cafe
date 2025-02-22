#ifndef WAIT_FOR_CONFIRMATION_CONDITION_HPP_
#define WAIT_FOR_CONFIRMATION_CONDITION_HPP_

#include <string>
#include <unordered_map>

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "robot_interfaces/srv/wait_for_confirmation.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/go_to_target.hpp"

namespace nav2_behavior_tree
{

class WaitForConfirmationCondition : public BT::ConditionNode
{

public:
  WaitForConfirmationCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);


  WaitForConfirmationCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("order", "Confirmation order to wait for kitchen, tables"),
      BT::InputPort<std::string>("timeout", "timeout to skip confirmation")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_;
  void detectArucoCallback(
      const robot_interfaces::srv::WaitForConfirmation::Response::SharedPtr response);

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<robot_interfaces::srv::WaitForConfirmation>::SharedPtr
    order_confirmation_client_;
  std::string order_;
  uint8_t timeout_;
  bool order_detected_;
};

}  // namespace nav2_behavior_tree
#endif
