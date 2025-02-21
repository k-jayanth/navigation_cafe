#include <memory>
#include <string>

#include "robot_behaviour_tree/navigate_to_target.hpp"

namespace nav2_behavior_tree
{
NavigateToTargetAction::NavigateToTargetAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf) 
    : BtActionNode<Action>(xml_tag_name, action_name, conf)
    {
    initialize_predefined_targets();
    }

void NavigateToTargetAction::on_tick()
{
  std::string target_name;
  if (!getInput("target", target_name)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToTargetAction: target not provided");
    return;
  }

  auto target_itr = predefined_targets.find(target_name);
  if (target_itr != predefined_targets.end()) {
    goal_.target_pose = target_itr->second;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToTargetAction: Invalid target '%s'", target_name.c_str());
    return;
  }
}

} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string &name,
        const BT::NodeConfiguration &config) {
    return std::make_unique<nav2_behavior_tree::NavigateToTargetAction>(
    name, "go_to_target", config);
    };
    factory.registerBuilder<nav2_behavior_tree::NavigateToTargetAction>("NavigateToTarget",
                                        builder);
}