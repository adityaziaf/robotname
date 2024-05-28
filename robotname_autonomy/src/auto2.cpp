#include "robotname_autonomy/plugins/action/get_intake_color.hpp"
#include "robotname_autonomy/plugins/service/set_intake_mechanism.hpp"
#include "robotname_autonomy/plugins/action/nav_to_pose.hpp"
#include "robotname_autonomy/plugins/action/nav_through_poses.hpp"
#include "robotname_autonomy/plugins/action/find_nearest_ball.hpp"
#include "robotname_autonomy/plugins/service/follow_path.hpp"
#include "robotname_autonomy/plugins/service/ball_available.hpp"
#include "robotname_autonomy/plugins/action/get_intake_proximity_array.hpp"
#include "robotname_autonomy/plugins/action/get_ball_grabbed.hpp"
#include "robotname_autonomy/plugins/action/flush_intake.hpp"
#include "robotname_autonomy/plugins/action/follow_ball.hpp"
#include "robotname_autonomy/plugins/service/get_nearest_ball.hpp"
#include "robotname_autonomy/plugins/subscriber/grab_nearest_ball.hpp"
#include "robotname_autonomy/plugins/subscriber/ball_grabbed.hpp"
#include "robotname_autonomy/plugins/service/amcl_update.hpp"
#include "robotname_autonomy/plugins/service/reset_ball_grabbed.hpp"
#include "robotname_autonomy/plugins/action/get_ball_grabbed_top.hpp"
#include "robotname_autonomy/plugins/service/rotate.hpp"
#include "robotname_autonomy/plugins/subscriber/intake_color.hpp"
#include "robotname_autonomy/plugins/subscriber/id_found.hpp"
#include "robotname_autonomy/plugins/service/get_current_pose.hpp"
#include "robotname_autonomy/plugins/subscriber/ball_grabbed_top.hpp"
#include "robotname_autonomy/plugins/action/wait_button.hpp"
#include "robotname_autonomy/plugins/action/rotate_speed.hpp"
#include "robotname_autonomy/plugins/action/move_with_lidar_reference.hpp"
#include "robotname_autonomy/plugins/service/set_tail_position.hpp"
#include "robotname_autonomy/plugins/action/setup_menu.hpp"
#include "robotname_autonomy/plugins/action/set_speed.hpp"

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
  pub_param.default_port_value = "/set_intake_mechanism";
  factory.registerNodeType<SetIntakeMechanism>("SetIntakeMechanism",pub_param);

  RosNodeParams nav_param;
  nav_param.nh = nh;
  nav_param.default_port_value = "/navigate_to_pose";
  nav_param.server_timeout = std::chrono::seconds(10);
  nav_param.wait_for_server_timeout = std::chrono::seconds(10);
  factory.registerNodeType<NavToPose>("NavToPose", nav_param);

  RosNodeParams navt_param;
  navt_param.nh = nh;
  navt_param.default_port_value = "/navigate_through_poses";
  navt_param.server_timeout = std::chrono::seconds(10);
  navt_param.wait_for_server_timeout = std::chrono::seconds(10);
  factory.registerNodeType<NavThroughPoses>("NavThroughPoses", navt_param);

  RosNodeParams follow;
  follow.nh = nh;
  follow.default_port_value = "/follow_path";
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

  RosNodeParams proximityarrayconfig;
  proximityarrayconfig.nh = nh;
  proximityarrayconfig.default_port_value = "/get_intake_proximity_array";
  factory.registerNodeType<GetIntakeProximityArray>("GetIntakeProximityArray", proximityarrayconfig);

  RosNodeParams ballgrabbedconfig;
  ballgrabbedconfig.nh = nh;
  ballgrabbedconfig.default_port_value = "/get_ball_grabbed";
  factory.registerNodeType<GetBallGrabbed>("GetBallGrabbed", ballgrabbedconfig);

  RosNodeParams flushconfig;
  flushconfig.nh = nh;
  flushconfig.default_port_value = "/flush_intake";
  factory.registerNodeType<FlushIntake>("FlushIntake", flushconfig);

  RosNodeParams followballconfig;
  followballconfig.nh = nh;
  followballconfig.default_port_value = "/follow_ball";
  factory.registerNodeType<Follow>("Follow", followballconfig);

  RosNodeParams getnearestballconfig;
  getnearestballconfig.nh = nh;
  getnearestballconfig.default_port_value = "/get_nearest_ball";
  factory.registerNodeType<GetNearestBall>("GetNearestBall", getnearestballconfig);

  RosNodeParams grabnearestballconfig;
  grabnearestballconfig.nh = nh;
  grabnearestballconfig.default_port_value = "/camera/objects/tracked";
  factory.registerNodeType<GrabNearestBall>("GrabNearestBall", grabnearestballconfig);

  RosNodeParams ballgrabbed;
  ballgrabbed.nh = nh;
  ballgrabbed.default_port_value = "/intake/detectedcolor";
  factory.registerNodeType<BallGrabbed>("BallGrabbed", ballgrabbed);

  RosNodeParams amclupdateocnfig;
  amclupdateocnfig.nh = nh;
  amclupdateocnfig.default_port_value = "/request_nomotion_update";
  factory.registerNodeType<AMCLUpdate>("AMCLUpdate", amclupdateocnfig);

  RosNodeParams resetball;
  resetball.nh = nh;
  resetball.default_port_value = "/reset_ball_grabbed";
  factory.registerNodeType<ResetBallGrabbed>("ResetBallGrabbed", resetball);

  RosNodeParams ballgrabtop;
  ballgrabtop.nh = nh;
  ballgrabtop.default_port_value = "/get_ball_grabbed_top";
  factory.registerNodeType<GetBallGrabbedTop>("GetBallGrabbedTop", ballgrabtop);

  RosNodeParams rotateconfig;
  rotateconfig.nh = nh;
  rotateconfig.default_port_value = "/rotate_server";
  factory.registerNodeType<Rotate>("Rotate", rotateconfig);

  RosNodeParams intakecolorconf;
  intakecolorconf.nh = nh;
  intakecolorconf.default_port_value = "/intake/detectedcolor";
  factory.registerNodeType<IntakeColor>("IntakeColor", intakecolorconf);

  RosNodeParams idfound;
  idfound.nh = nh;
  idfound.default_port_value = "/idfound";
  factory.registerNodeType<IdFound>("IdFound", idfound);

  RosNodeParams getcurrentpose;
  getcurrentpose.nh = nh;
  getcurrentpose.default_port_value = "/get_current_pose_server";
  factory.registerNodeType<GetCurrentPose>("GetCurrentPose", getcurrentpose);

  RosNodeParams ballgrabbedtop;
  ballgrabbedtop.nh = nh;
  ballgrabbedtop.default_port_value = "/intake2/detectedcolor";
  factory.registerNodeType<BallGrabbedTop>("BallGrabbedTop", ballgrabbedtop);

  RosNodeParams waitbutton;
  waitbutton.nh = nh;
  waitbutton.default_port_value = "/wait_button";
  factory.registerNodeType<WaitButton>("WaitButton", waitbutton);

  RosNodeParams rotateconf;
  rotateconf.nh = nh;
  rotateconf.default_port_value = "/rotate_speed";
  factory.registerNodeType<RotateSpeed>("RotateSpeed", rotateconf);

  RosNodeParams movewithlidarconf;
  movewithlidarconf.nh = nh;
  movewithlidarconf.default_port_value = "/move_with_lidar_reference";
  factory.registerNodeType<MoveWithLidarReference>("MoveWithLidarReference", movewithlidarconf);

  RosNodeParams settailposition;
  settailposition.nh = nh;
  settailposition.default_port_value = "/set_tail_position";
  factory.registerNodeType<SetTailPosition>("SetTailPosition", settailposition);

  RosNodeParams menuconfig;
  menuconfig.nh = nh;
  menuconfig.default_port_value = "/setup_menu";
  factory.registerNodeType<SetupMenu>("SetupMenu", menuconfig);

  RosNodeParams setspeedconfig;
  setspeedconfig.nh = nh;
  setspeedconfig.default_port_value = "/set_speed";
  factory.registerNodeType<SetSpeed>("SetSpeed", setspeedconfig);

  auto tree = factory.createTreeFromFile(default_bt_xml_file);



  BT::NodeStatus status;
  while(!BT::isStatusCompleted(status)) 
{
  status = tree.tickWhileRunning();
  std::cout << "--- Tree status: " << toStr(status) << " ---\n\n";
  tree.sleep(std::chrono::milliseconds(10));
}
  

  return 0;
}