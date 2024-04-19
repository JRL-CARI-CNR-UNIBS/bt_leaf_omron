#ifndef GO_TO_GOAL_HPP
#define GO_TO_GOAL_HPP

#include <behaviortree_ros2/bt_service_node.hpp>
#include <omron_msgs/srv/goto_goal.hpp>
#include <cnr_param/cnr_param.h>

using GoToGoal = omron_msgs::srv::GotoGoal;

class OmronGoToGoal : public BT::RosServiceNode<GoToGoal>
{
public:
  OmronGoToGoal(const std::string& name,
               const BT::NodeConfig& conf,
               const BT::RosNodeParams& params)
    : RosServiceNode<GoToGoal>(name, conf, params)
  {
    auto param_ns = getInput<std::string>("param_ns");
    ns_ = "/bt_executer/" + param_ns.value();
  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static BT::PortsList providedPorts();

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(Request::SharedPtr& request) override;

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

protected:
  std::string goal;
  std::string ns_;
};

#endif // GO_TO_GOAL_HPP
