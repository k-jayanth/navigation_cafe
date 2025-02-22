#include "robot_behaviour_tree/wait_for_confirmation.hpp"
#include "robot_interfaces/action/go_to_target.hpp"
#include "robot_interfaces/srv/wait_for_confirmation.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot_behaviour_tree
{
WaitForConfirmation::WaitForConfirmation(const rclcpp::NodeOptions &options)
    : Node("wait_for_confirmation_node", options), navigate_status_(ActionStatus::UNKNOWN)
{
    RCLCPP_DEBUG(this->get_logger(), "Service server 'wait_for_confirmation' started.");

  // Initialize the action client
  client_ptr_ = rclcpp_action::create_client<ActionT>(this, "go_to_target");

  // Initialize the service server
  service_ = this->create_service<ServiceT>("order_confirmation", 
    [this](const std::shared_ptr<ServiceT::Request> request,
           std::shared_ptr<ServiceT::Response> response) {
        this->handleServiceRequest(request, response);
    });

  RCLCPP_INFO(this->get_logger(), "Service server 'wait_for_confirmation' started.");
}

WaitForConfirmation::~WaitForConfirmation() {
    RCLCPP_INFO(this->get_logger(), "Service server 'wait_for_confirmation' existing.");
}

void WaitForConfirmation::handleServiceRequest(
    const std::shared_ptr<ServiceT::Request> request,
    std::shared_ptr<ServiceT::Response> response) {

  current_order_ = request->order;
  timeout_ = request->timeout;

  // Split the order string by spaces to get the list of targets.
  std::vector<std::string> targets;
  std::istringstream order_stream(current_order_);
  std::string target;
  while (order_stream >> target) {
    targets.push_back(target);
  }

  // Iterate through targets and call `goToTarget` for each.
  for (const auto& target : targets) {
    RCLCPP_INFO(get_logger(), "Processing target: %s", target.c_str());
    goToTarget(target);

    // Wait for confirmation or timeout.
    auto start_time = this->get_clock()->now();
    while (rclcpp::ok()) {
      auto elapsed_time = this->get_clock()->now() - start_time;
      if (elapsed_time.seconds() > timeout_) {
        RCLCPP_WARN(get_logger(), "Timeout reached, moving to next target.");
        break;
      }
      if (navigate_status_ == ActionStatus::SUCCEEDED) {
        RCLCPP_INFO(get_logger(), "Navigation to %s succeeded.", target.c_str());
        break;
      }
    }
  }

  response->success = (navigate_status_ == ActionStatus::SUCCEEDED);
}

void WaitForConfirmation::goToTarget(const std::string& target) {

    robot_interfaces::action::GoToTarget::Goal navigate_goal;
    navigate_goal.target = target;

    auto goal_options = rclcpp_action::Client<robot_interfaces::action::GoToTarget>::SendGoalOptions();
    goal_options.goal_response_callback =
        std::bind(&WaitForConfirmation::goalResponseCallback, this, std::placeholders::_1);
    goal_options.result_callback =
        std::bind(&WaitForConfirmation::resultCallback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(navigate_goal, goal_options);

}

void WaitForConfirmation::resultCallback(
    const rclcpp_action::ClientGoalHandle<robot_interfaces::action::GoToTarget>::WrappedResult &result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "Navigation to target succeeded");
    navigate_status_ = ActionStatus::SUCCEEDED;
  } else {
    RCLCPP_ERROR(get_logger(), "Navigation to target failed");
    navigate_status_ = ActionStatus::FAILED;
  }
}

void WaitForConfirmation::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<robot_interfaces::action::GoToTarget>::SharedPtr &goal) {
  if (!goal) {
    RCLCPP_ERROR(get_logger(), "Navigation server rejected goal");
  } else {
    RCLCPP_INFO(get_logger(), "Navigation server accepted goal");
  }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_behaviour_tree::WaitForConfirmation)