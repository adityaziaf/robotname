#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "robotname_autonomy/plugins/follow_path.hpp"
#include "robotname_autonomy/utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/utils.h"

PortsList FollowPose::providedPorts()
  {
    return { InputPort<std::string>("frame_id"), InputPort<Pose>("goal")};
  }
  
bool FollowPose::setMessage(geometry_msgs::msg::PoseStamped &msg)
{
    Pose targetpose;
    auto frame = getInput<std::string>("frame_id");
    getInput("goal", targetpose);
    
    msg.header.frame_id = frame.value();
    msg.pose.position.x = targetpose.x;
    msg.pose.position.y = targetpose.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, targetpose.theta);
    q.normalize();
    msg.pose.orientation = tf2::toMsg(q);
   
    
    return true;
}


