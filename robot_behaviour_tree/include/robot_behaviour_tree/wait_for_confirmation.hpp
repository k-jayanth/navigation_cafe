#ifndef WAIT_FOR_CONFIRMATION_HPP_
#define WAIT_FOR_CONFIRMATION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_interfaces/srv/wait_for_confirmation.hpp"
#include "robot_interfaces/action/go_to_target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

enum class ActionStatus {
    UNKNOWN = 0,
    PROCESSING = 1,
    FAILED = 2,
    SUCCEEDED = 3,
    CANCELED = 4
};
namespace robot_behaviour_tree
{
class WaitForConfirmation : public rclcpp::Node {
 public:
  using ActionT = robot_interfaces::action::GoToTarget;
  using ServiceT = robot_interfaces::srv::WaitForConfirmation;

  WaitForConfirmation(const rclcpp::NodeOptions &options);
  ~WaitForConfirmation();

 protected:
  void handleServiceRequest(const std::shared_ptr<ServiceT::Request> request,
                            std::shared_ptr<ServiceT::Response> response);

  void goToTarget(const std::string& target);
  void resultCallback(const rclcpp_action::ClientGoalHandle<robot_interfaces::action::GoToTarget>::WrappedResult &result);
  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<robot_interfaces::action::GoToTarget>::SharedPtr &goal);

 private:
 rclcpp_action::Client<ActionT>::SharedPtr client_ptr_;
 rclcpp::Service<ServiceT>::SharedPtr service_;
  
  ActionStatus navigate_status_;
  std::string current_order_;
  int timeout_;
};
}
#endif  // WAIT_FOR_CONFIRMATION_HPP_
