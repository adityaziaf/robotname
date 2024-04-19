#include "robotname_autonomy/plugins/get_intake_color.hpp"
#include "robotname_autonomy/plugins/set_intake_mechanism.hpp"
#include "robotname_autonomy/plugins/nav_to_pose.hpp"
#include "robotname_autonomy/plugins/nav_through_poses.hpp"


#include "ament_index_cpp/get_package_share_directory.hpp"

const std::string default_bt_xml_file = 
    ament_index_cpp::get_package_share_directory("robotname_autonomy") + "/bt_xml/findobject.xml";

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("autonomy_node");

  BehaviorTreeFactory factory;

  RosNodeParams params;
  params.nh = nh;
  params.default_port_value = "/intake/detectedcolor";
  factory.registerNodeType<ReceiveString>("ReceiveString", params);

  RosNodeParams pub_param;
  pub_param.nh = nh;
  pub_param.default_port_value = "/set_mekanism";
  factory.registerNodeType<SetIntakeMechanism>("SetIntakeMechanism",pub_param);

  RosNodeParams nav_param;
  nav_param.nh = nh;
  nav_param.default_port_value = "/navigate_to_pose";
  factory.registerNodeType<NavToPose>("NavToPose", nav_param);

  RosNodeParams navt_param;
  navt_param.nh = nh;
  navt_param.default_port_value = "/navigate_through_poses";
  factory.registerNodeType<NavThroughPoses>("NavThroughPoses", navt_param);

  auto tree = factory.createTreeFromFile(default_bt_xml_file);

  BT::NodeStatus ret;

  while(ret != BT::NodeStatus::SUCCESS)
  {
    ret = tree.tickOnce();
    rclcpp::spin_some(nh);
    tree.sleep(10ms);
  }

  return 0;
}