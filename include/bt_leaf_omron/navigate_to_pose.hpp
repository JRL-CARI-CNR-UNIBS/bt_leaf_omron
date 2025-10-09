#ifndef NAVIGATE_TO_POSE_HPP
#define NAVIGATE_TO_POSE_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "omron_msgs/action/goto_goal.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using GotoGoalAction = omron_msgs::action::GotoGoal;

namespace BT
{

template<>
inline std::vector<double> convertFromString(StringView str)
{
  std::vector<double> vec;
  auto parts = splitString(str, ';');
  for (auto & element: parts) {
    vec.push_back(convertFromString<double>(element));
  }
  return vec;
}

}  // namespace BT


class OmronNavigateToPose : public BT::RosActionNode<GotoGoalAction>
{
public:
  OmronNavigateToPose(const std::string& name,
                            const BT::NodeConfig& conf,
                            const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_target"),
        BT::InputPort<std::string>("frame_id"),
        BT::InputPort<std::vector<double>>("position"),
        BT::InputPort<std::vector<double>>("orientation"),
      }
    );
  }

  bool setGoal(Goal & goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(
    const std::shared_ptr<const GotoGoalAction::Feedback> feedback) override;

// protected:
//   std::string goal;
//   std::string ns_;
};

#endif // NAVIGATE_TO_POSE_HPP
