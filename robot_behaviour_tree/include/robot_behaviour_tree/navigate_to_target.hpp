#ifndef NAVIGATE_TO_TARGET_ACTION_HPP_
#define NAVIGATE_TO_TARGET_ACTION_HPP_

#include <string>
#include <unordered_map>

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "robot_interfaces/action/go_to_target.hpp"

namespace nav2_behavior_tree
{

class NavigateToTargetAction : public BtActionNode <robot_interfaces::action::GoToTarget>
{
  using Action = robot_interfaces::action::GoToTarget;
  using ActionResult = Action::Result;
  using ActionGoal = Action::Goal;

public:
  NavigateToTargetAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("target", "Destination to plan to")
      });
  }

  std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> predefined_targets;

  void initialize_predefined_targets()
  {
    predefined_targets["home"] = create_pose_stamped(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    predefined_targets["table1"] = create_pose_stamped(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    predefined_targets["table2"] = create_pose_stamped(2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    predefined_targets["table3"] = create_pose_stamped(3.0, 4.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  }

private:
  geometry_msgs::msg::PoseStamped create_pose_stamped(
    double x, double y, double z,
    double orientation_x, double orientation_y,
    double orientation_z, double orientation_w)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;
    pose_stamped.pose.position.z = z;
    pose_stamped.pose.orientation.x = orientation_x;
    pose_stamped.pose.orientation.y = orientation_y;
    pose_stamped.pose.orientation.z = orientation_z;
    pose_stamped.pose.orientation.w = orientation_w;
    return pose_stamped;
  }
};

}  // namespace robot_behavior_tree
#endif
