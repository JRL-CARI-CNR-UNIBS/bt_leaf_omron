#include <bt_leaf_omron/navigate_to_pose.hpp>

OmronNavigateToPose::OmronNavigateToPose(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: RosActionNode<GotoGoalAction>(name, conf, params)
{
}

bool OmronNavigateToPose::setGoal(RosActionNode::Goal & goal)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "OmronNavigateToPose ticked.");

  auto pose_target = getInput<geometry_msgs::msg::PoseStamped>("pose_target");
  if (pose_target) {
    goal.goal = pose_target.value();
    return true;
  }

  // If pose_target is not set, build it from components
  geometry_msgs::msg::PoseStamped pose;

  // Required fields: frame_id, position (3), orientation (4)
  if (!getInput("frame_id", pose.header.frame_id)) {
    throw BT::RuntimeError("Missing parameter [frame_id]");
  }

  std::vector<double> position;
  if (!getInput("position", position) || position.size() != 3) {
    throw BT::RuntimeError("Invalid or missing parameter [position]. Expected 3 elements");
  }

  std::vector<double> orientation;
  if (!getInput("orientation", orientation) || orientation.size() != 4) {
    throw BT::RuntimeError("Invalid or missing parameter [orientation]. Expected 4 elements");
  }

  pose.header.stamp = node_.lock()->now();  // timestamp current time
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation.x = orientation[0];
  pose.pose.orientation.y = orientation[1];
  pose.pose.orientation.z = orientation[2];
  pose.pose.orientation.w = orientation[3];

  goal.goal = pose;

  return true;
}

BT::NodeStatus OmronNavigateToPose::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "%s: onResultReceived", name().c_str());
  RCLCPP_INFO(node_.lock()->get_logger(), "%s Result: %d", name().c_str(), (wr.result->result));
  int code = wr.result->result;

  if(code != omron_msgs::action::GotoGoal::Result::ARRIVED)
  {
    RCLCPP_INFO(node_.lock()->get_logger(), "%s failed with error code: %d (2 = FAILED)", name().c_str(), code);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus OmronNavigateToPose::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus OmronNavigateToPose::onFeedback(
  const std::shared_ptr<const GotoGoalAction::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class OmronNavigateToPose will self register with name  "OmronNavigateToPose".
CreateRosNodePlugin(OmronNavigateToPose, "OmronNavigateToPose");