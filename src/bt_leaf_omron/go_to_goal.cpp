#include <bt_leaf_omron/go_to_goal.hpp>
#include <behaviortree_ros2/plugins.hpp>

/**
 * @brief Tokenize a string, using a/multiple delimiters
 *
 * @param node The node where the log will be printed
 * @param ns The namespace of the parameter
 * @param param_name The name of the parameter
 * @param param The output parameter
 * @param what The description of the output of the parameter loader
 * @return bool True if the parameter is loaded correctly, false otherwise
 */
template<typename T>
inline bool get_param(rclcpp::Node *node, std::string ns, std::string param_name, T& param, std::string what)
{
  if(cnr::param::has(ns + param_name, what))
  {
    if(not cnr::param::get(ns + param_name, param, what))
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot load " << ns + param_name + " parameter.");
      RCLCPP_ERROR_STREAM(node->get_logger(), "what:" << what);
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), ns + param_name + " parameter not available.");
    RCLCPP_ERROR_STREAM(node->get_logger(), "what: " << what);
    return false;
  }
  return true;
}

BT::PortsList OmronGoToGoal::providedPorts()
{
  return providedBasicPorts({BT::InputPort<std::string>("param_ns")});
}

bool OmronGoToGoal::setRequest(Request::SharedPtr& request)
{
    // Get required parameters
  std::string w;
  get_param(node_.get(), ns_, "/goal", goal, w);

  // getInput("goal", request->goal); // send request to service
  request->goal = goal;
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
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown Error: " << response->result);
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus OmronGoToGoal::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
  return BT::NodeStatus::SUCCESS;
}

CreateRosNodePlugin(OmronGoToGoal, "OmronGoToGoalSkill");
