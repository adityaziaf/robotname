#include <behaviortree_ros2/bt_service_node.hpp>
#include "robotname_msgs/srv/get_best_silo.hpp"
using namespace BT;


class GetBestSilo: public RosServiceNode<robotname_msgs::srv::GetBestSilo>
{
  public:

  GetBestSilo(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosServiceNode<robotname_msgs::srv::GetBestSilo>(name, conf, params)
  {}

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosServiceNode::providedBasicPorts()
  static BT::PortsList providedPorts();

  // This is called when the TreeNode is ticked and it should
  // send the request to the service provider
  bool setRequest(robotname_msgs::srv::GetBestSilo::Request::SharedPtr& request) override;

  // Callback invoked when the answer is received.
  // It must return SUCCESS or FAILURE
  NodeStatus onResponseReceived(const robotname_msgs::srv::GetBestSilo::Response::SharedPtr& response) override;

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};