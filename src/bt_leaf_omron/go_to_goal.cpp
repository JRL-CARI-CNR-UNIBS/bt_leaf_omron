#include <bt_leaf_omron/go_to_goal.hpp>

BT::PortsList OmronGoToGoal::providedPorts()
{
  return providedBasicPorts({BT::InputPort<unsigned>("goal")});
}

bool OmronGoToGoal::setRequest(Request::SharedPtr& request)
{
  getInput("goal", request->goal); // send request to service
  goal = request->goal;
  return true;
}

BT::NodeStatus OmronGoToGoal::onResponseReceived(const Response::SharedPtr& response)
{
  if(response->result == omron_msgs::srv::GotoGoal::Response::ARRIVED)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Arrived to goal [%s]", this->goal.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  else if (response->result == omron_msgs::srv::GotoGoal::Response::FAILED)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error during movement to goal [%s]", this->goal.c_str());
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown Error: " << response->result);
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus OmronGoToGoal::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}
