#include <memory>
#include <string>

#include "robot_behaviour_tree/wait_for_confirmation_condition.hpp"

namespace nav2_behavior_tree
{
WaitForConfirmationCondition::WaitForConfirmationCondition(const std::string &condition_name,
    const BT::NodeConfiguration &conf)
  : BT::ConditionNode(condition_name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    order_confirmation_client_  =
        node_->create_client<robot_interfaces::srv::WaitForConfirmation>(
            "/order_confirmation");

    while (!order_confirmation_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    }

    // Variables
    this->order_detected_ = false;
    order_ = getInput<std::string>("order").value();
    timeout_ =
        static_cast<uint8_t>(std::stoi(getInput<std::string>("timeout").value()));
  }

BT::NodeStatus WaitForConfirmationCondition::tick()
{
  auto request =
      std::make_shared<robot_interfaces::srv::WaitForConfirmation::Request>();
  request->order  = order_;
  request->timeout = timeout_;

  auto future = order_confirmation_client_->async_send_request(request);
  executor_.spin_until_future_complete(future);

  order_detected_ = future.get()->success;

  if (order_detected_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;

  return BT::NodeStatus::SUCCESS;
}


} 

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::WaitForConfirmationCondition>(
      "WaitForConfirmation");
}