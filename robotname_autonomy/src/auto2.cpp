#include "robotname_autonomy/plugins/get_intake_color.hpp"
#include "robotname_autonomy/plugins/set_intake_mechanism.hpp"
#include "robotname_autonomy/plugins/nav_to_pose.hpp"
#include "robotname_autonomy/plugins/nav_through_poses.hpp"
#include "robotname_autonomy/plugins/find_nearest_ball.hpp"
#include "robotname_autonomy/plugins/follow_path.hpp"
#include "robotname_autonomy/plugins/ball_available.hpp"
#include "robotname_autonomy/plugins/set_speed.hpp"


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
  params.default_port_value = "/get_intake_color";
  factory.registerNodeType<GetIntakeColor>("GetIntakeColor", params);


  RosNodeParams pub_param;
  pub_param.nh = nh;
  pub_param.default_port_value = "/set_mekanism";
  factory.registerNodeType<SetIntakeMechanism>("SetIntakeMechanism",pub_param);

  RosNodeParams nav_param;
  nav_param.nh = nh;
  nav_param.default_port_value = "/navigate_to_pose";
  nav_param.server_timeout = std::chrono::seconds(5);
  nav_param.wait_for_server_timeout = std::chrono::seconds(10);
  factory.registerNodeType<NavToPose>("NavToPose", nav_param);

  RosNodeParams navt_param;
  navt_param.nh = nh;
  navt_param.default_port_value = "/navigate_through_poses";
  navt_param.server_timeout = std::chrono::seconds(5);
  navt_param.wait_for_server_timeout = std::chrono::seconds(10);
  factory.registerNodeType<NavThroughPoses>("NavThroughPoses", navt_param);

  RosNodeParams follow;
  follow.nh = nh;
  follow.default_port_value = "/goal_pose";
  follow.server_timeout = std::chrono::seconds(5);
  follow.wait_for_server_timeout = std::chrono::seconds(10);
  factory.registerNodeType<FollowPose>("FollowPose", follow);

  RosNodeParams detectball;
  detectball.nh = nh;
  detectball.default_port_value = "/find_nearest_ball";
  factory.registerNodeType<FindNearestBall>("FindNearestBall", detectball);

  RosNodeParams checkball;
  checkball.nh = nh;
  checkball.default_port_value = "/ball_available";
  factory.registerNodeType<BallAvailable>("BallAvailable", checkball);

  RosNodeParams setspeed;
  setspeed.nh = nh;
  setspeed.default_port_value = "/set_speed";
  factory.registerNodeType<SetSpeed>("SetSpeed", setspeed);


  auto tree = factory.createTreeFromFile(default_bt_xml_file);

  BT::NodeStatus status;
  //status = tree.tickOnce();
  // while(status == NodeStatus::RUNNING || status == NodeStatus::FAILURE) 
  // {
  //   // Sleep to avoid busy loops.
  //   // do NOT use other sleep functions!
  //   // Small sleep time is OK, here we use a large one only to
  //   // have less messages on the console.
  //   tree.sleep(std::chrono::milliseconds(10));
  //   //std::cout << "--- Tree ticking ---\n";
  //   status = tree.tickOnce();
  //   std::cout << "--- Tree status: " << toStr(status) << " ---\n\n";
  // }
  while(!BT::isStatusCompleted(status)) 
{
  status = tree.tickOnce();
  std::cout << "--- Tree status: " << toStr(status) << " ---\n\n";
  tree.sleep(std::chrono::milliseconds(10));
}
  

  return 0;
}