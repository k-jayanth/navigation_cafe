// ros2 action send_goal /goto_home roast_interfaces/action/GoToHome ""

#include "robot_behaviour_tree/go_to_target.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp_components/register_node_macro.hpp"

GoToTargetActionServer::GoToTargetActionServer(const rclcpp::NodeOptions &options)
    : Node("go_to_target_action_server", options) {
  // Declare parameters
//   declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);
//   declare_parameter("home_backup_distance", 1.0);
//   declare_parameter("home_spin_angle", 3.14159);
  declare_parameter("target", "home");

  // Configure
//   stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();
//   backup_distance_ = get_parameter("home_backup_distance").as_double();
//   spin_angle_ = get_parameter("home_spin_angle").as_double();
  target_ = get_parameter("target").as_string();

  // Action Clients
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateClientT>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "navigate_to_pose", callback_group_);

  // Action Server
  action_server_ = std::make_unique<ActionServer>(
      get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_waitables_interface(), "go_to_target",
      std::bind(&GoToTargetActionServer::goToTarget, this));

  action_server_->activate();
  RCLCPP_INFO(get_logger(), "Go To Target action server started");
}

GoToTargetActionServer::~GoToTargetActionServer() {
  action_server_->terminate_all();
}

void GoToTargetActionServer::goToTarget() {
  rclcpp::Rate loop_rate(loop_rate_);
  goal_ = action_server_->get_current_goal();
  target_ = goal_->target;

  // Check if request is valid
  if (!action_server_->is_server_active()) {
    RCLCPP_ERROR(get_logger(), "Action server is inactive. Rejecting goal.");
    result_->success = false;
    action_server_->terminate_current(result_);
    return;
  }

  // Cancel any previous goals
  if (nav_to_pose_client_->action_server_is_ready()) {
    nav_to_pose_client_->async_cancel_all_goals();
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  if(target_ == "home"){
    RCLCPP_INFO(get_logger(), "Navigating to home");

    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();
    goal_pose.pose.position.x = -0.226;
    goal_pose.pose.position.y = -2.1;
    goal_pose.pose.position.z = 0.0;

    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 1.0;
    goal_pose.pose.orientation.w = 0.0;
  }
  else if(target_ == "kitchen"){
    RCLCPP_INFO(get_logger(), "Navigating to kitchen");

    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();
    goal_pose.pose.position.x = 0.92;
    goal_pose.pose.position.y = -1.87;
    goal_pose.pose.position.z = 0.0;

    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 1.0;
    goal_pose.pose.orientation.w = 0.0;
  }
  else if(target_ == "table1"){
    RCLCPP_INFO(get_logger(), "Navigating to table1");

    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();
    goal_pose.pose.position.x = 0.536;
    goal_pose.pose.position.y = 0.159;
    goal_pose.pose.position.z = 0.0;

    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 1.0;
    goal_pose.pose.orientation.w = 0.0;
  }
  else if(target_ == "table2"){
    RCLCPP_INFO(get_logger(), "Navigating to table2");

    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();
    goal_pose.pose.position.x = 1.84;
    goal_pose.pose.position.y = 1.41;
    goal_pose.pose.position.z = 0.0;

    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 1.0;
    goal_pose.pose.orientation.w = 0.0;
  }
  else if(target_ == "table3"){
    RCLCPP_INFO(get_logger(), "Navigating to table3");

    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();
    goal_pose.pose.position.x = -0.92;
    goal_pose.pose.position.y = -1.02;
    goal_pose.pose.position.z = 0.0;

    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 1.0;
    goal_pose.pose.orientation.w = 0.0;
  }
  else{
    RCLCPP_INFO(get_logger(), "Navigating to kitchen");

    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();
    goal_pose.pose.position.x = 0.92;
    goal_pose.pose.position.y = -1.87;
    goal_pose.pose.position.z = 0.0;

    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 1.0;
    goal_pose.pose.orientation.w = 0.0;
  }

  this->navigateToTarget(goal_pose);

  if (current_goal_status_ == ActionStatus::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "GoToTarget action succeeded");
    auto result = std::make_shared<ActionT::Result>();
    result->success = true;
    action_server_->succeeded_current(result);
    return;
  } else {
    RCLCPP_ERROR(get_logger(), "Navigation failed. Stopping.");
    auto result = std::make_shared<ActionT::Result>();
    result->success = false;
    action_server_->terminate_current(result);
    return;
  }
}

void GoToTargetActionServer::navigateToTarget(
    geometry_msgs::msg::PoseStamped goal_pose) {
  NavigateClientT::Goal navigate_goal;
  auto feedback = std::make_shared<NavigateClientT::Feedback>();
  auto result = std::make_shared<NavigateClientT::Result>();
  new_goal_ = true;

  while (rclcpp::ok()) {
    // Check if asked to process another action
    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the previous goal");
      goal_ = action_server_->accept_pending_goal();
      new_goal_ = true;
      continue;
    }

    if (new_goal_) {
      RCLCPP_INFO(get_logger(), "Processing new goal");

      navigate_goal.pose = goal_pose;

      auto new_goal_options =
          rclcpp_action::Client<NavigateClientT>::SendGoalOptions();
      new_goal_options.goal_response_callback =
          std::bind(&GoToTargetActionServer::goalResponseCallback, this,
                    std::placeholders::_1);
      new_goal_options.result_callback = std::bind(
          &GoToTargetActionServer::resultCallback, this, std::placeholders::_1);

      future_goal_handle_ =
          nav_to_pose_client_->async_send_goal(navigate_goal, new_goal_options);
      navigate_status_ = ActionStatus::PROCESSING;

      new_goal_ = false;
    }

    if (navigate_status_ == ActionStatus::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      current_goal_status_ = ActionStatus::SUCCEEDED;
      break;
    } else if (navigate_status_ == ActionStatus::CANCELED ||
               navigate_status_ == ActionStatus::FAILED) {
      RCLCPP_ERROR(get_logger(), "Navigation failed");
      current_goal_status_ = ActionStatus::FAILED;
      auto result = std::make_shared<ActionT::Result>();
      result->success = false;
      action_server_->terminate_current(result);
      return;
    }
  }
}

void GoToTargetActionServer::resultCallback(
    const rclcpp_action::ClientGoalHandle<NavigateClientT>::WrappedResult
        &result) {
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto action_result = std::make_shared<ActionT::Result>();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Navigation server succeeded");
      navigate_status_ = ActionStatus::SUCCEEDED;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Navigation server aborted");
      navigate_status_ = ActionStatus::FAILED;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Navigation server canceled");
      navigate_status_ = ActionStatus::CANCELED;
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Navigation server failed");
      navigate_status_ = ActionStatus::FAILED;
      break;
  }
}

void GoToTargetActionServer::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr &goal) {
  if (!goal) {
    RCLCPP_ERROR(get_logger(), "Navigation server rejected goal");
    action_server_->terminate_all();
  } else {
    RCLCPP_INFO(get_logger(), "Navigation server accepted goal");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(GoToTargetActionServer)
