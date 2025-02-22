#ifndef GO_TO_TARGET_HPP_
#define GO_TO_TARGET_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_interfaces/action/go_to_target.hpp"

enum class ActionStatus {
    UNKNOWN = 0,
    PROCESSING = 1,
    FAILED = 2,
    SUCCEEDED = 3,
    CANCELED = 4
};

class GoToTargetActionServer : public rclcpp::Node {
 public:
  using ActionT = robot_interfaces::action::GoToTarget;
  using NavigateClientT = nav2_msgs::action::NavigateToPose;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;
  using NavigateClient = rclcpp_action::Client<NavigateClientT>;

  GoToTargetActionServer(const rclcpp::NodeOptions &options);
  ~GoToTargetActionServer();

 protected:
  void goToTarget();

  void resultCallback(
      const rclcpp_action::ClientGoalHandle<NavigateClientT>::WrappedResult
          &result);
  void goalResponseCallback(
      const rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr &goal);

  // Our action server
  std::unique_ptr<ActionServer> action_server_;
  NavigateClient::SharedPtr nav_to_pose_client_;
  std::shared_future<
      rclcpp_action::ClientGoalHandle<NavigateClientT>::SharedPtr>
      future_goal_handle_;

 private:
  void navigateToTarget(geometry_msgs::msg::PoseStamped goal_pose);
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  bool new_goal_;
  ActionStatus current_goal_status_, navigate_status_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  int loop_rate_;
  std::string target_;

  ActionT::Result::SharedPtr result_;
  ActionT::Goal::ConstSharedPtr goal_;
};

#endif  // GO_TO_TARGET_HPP_
