#include "robotname_hardware/robot_node.hpp"

#include <bitset>
#include <vector>

#include "robotname_hardware/udp.hpp"

#define IP_SERVER "192.168.1.111" //pc
#define IP_CLIENT "192.168.1.134" //stm
#define PORT 12345

UDP n_udp(IP_SERVER, PORT);

// using namespace std::chrono_literals;

#define NUM_SWERVE 3
#define BTN_X 1
#define BTN_O 2
#define BTN_SQ 8
#define BTN_TR 4
#define BTN_R1 32
#define BTN_R3 4096
#define BTN_L1 16
#define BTN_L3 2048
#define BTN_RIGHT 65536
#define BTN_UP 8192
#define BTN_LEFT 32768
#define BTN_DOWN 16384
#define BTN_R2 128
#define BTN_L2 64
#define BTN_SHARE 256
#define BTN_OPT 512
#define BTN_PS 1024

#define LIFT_SPEED_UP 33 //R1+X
#define LIFT_SPEED_DOWN 34 //R1+O
#define LIFT_OFF 18 //L1+O
#define DRIB_SPEED_UP 40 //R1+KOTAK
#define DRIB_SPEED_DOWN 36 //R1+SGTIGA
#define DRIB_OFF 20 //L1+sgt

#define LIFT_MAX_SPEED 3
#define DRIB_MAX_SPEED 3

uint8_t drib_flag = 0;
uint8_t lift_flag = 0;

struct send_to_robot {
  uint32_t timestamp;

  struct {
    float x;
    float y;
    float theta;
  } body_speed;

  union {
    struct {
      uint8_t reset_odom : 1;
    };
    uint32_t flag;
  };

  struct{
	  float lift_speed;
    float drib_speed;
  }ball_drib;
};

struct recv_from_robot {
  uint32_t timestamp;

  struct {
    float x;
    float y;
    float theta;
  } g_position;

  struct {
    float x;
    float y;
    float theta;
  } body_speed;

  float linear_speed[3];
  float angular_speed[3];
  union {
    struct {
      uint8_t odom_sensor : 1;
    };
    uint16_t sensor_status;
  };

  float v_bat;
  uint32_t global_flag;
};

send_to_robot send_data;
recv_from_robot recv_data;

robotNode::robotNode() : Node("robot_node") {
  /*IP Config*/
  n_udp.init();
  n_udp.setClient(IP_CLIENT, PORT);
  /*Start Timer*/ //std::chrono::microseconds)100
  udp_timer = this->create_wall_timer((std::chrono::milliseconds)1,
                                      std::bind(&robotNode::udpLoop, this));
  /*Start Publisher*/
  // g_position_pub          =
  // this->create_publisher<geometry_msgs::msg::Pose2D>(
  //                           "global_position", 10);
  // body_speed_pub          =
  // this->create_publisher<geometry_msgs::msg::Twist>(
  //                           "body_speed", 10);

// joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy",10,std::bind());

  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history,
                                rmw_qos_profile_sensor_data.history),
      rmw_qos_profile_sensor_data);

  /* Publisher */
  odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

  imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("odom/imu", qos);

  cmd_base_vel = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&robotNode::speedSub, this, std::placeholders::_1));

  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy",10,std::bind(&robotNode::joy_callback,this,std::placeholders::_1));

  mekanism_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "set_mekanism",10,std::bind(&robotNode::set_mekanism,this,std::placeholders::_1));

  /* Subscriber */

  // run_status_sub          = this->create_subscription<std_msgs::msg::Bool> (
  //                           "run_status", 10,
  //                           std::bind(&robotNode::robot_status_callback,
  //                           this, std::placeholders::_1));
  // seedling_sub            =
  // this->create_subscription<std_msgs::msg::Float32MultiArray> (
  //                           "seedling", 10,
  //                           std::bind(&robotNode::gripper_callback, this,
  //                           std::placeholders::_1));
  // odom_reset_sub          = this->create_subscription<std_msgs::msg::Bool> (
  //                           "odometry/reset", 10,
  //                           std::bind(&robotNode::odomResetSub, this,
  //                           std::placeholders::_1));
  odom_reset_service = this->create_service<std_srvs::srv::Empty>(
      "odom/reset", std::bind(&robotNode::odomResetSub, this,
                              std::placeholders::_1, std::placeholders::_2));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void robotNode::set_mekanism(const std_msgs::msg::Float32MultiArray &msg)
{
  send_data.ball_drib.drib_speed = msg.data.at(1);
  send_data.ball_drib.lift_speed = msg.data.at(0);

}
void robotNode::joy_callback(const sensor_msgs::msg::Joy &msg){
    auto axes = msg.axes;
    auto buttons = msg.buttons;
    float analog[6];
    int button_t;

    std::cout << "Axes: ";
    int8_t i = 0;
    for (auto axis : axes) {
        analog[i] = axis;
        i++;
        std::cout << axis << " ";
    }
    std::cout << std::endl;

    std::cout << "Buttons: ";
    for (auto button : buttons) {
        std::cout << button << " ";
        button_t = button;
    }
    std::cout << std::endl;

    float analog_lx = analog[0];
    float analog_ly = analog[1];
    float analog_rx = analog[3];
    float analog_ry = analog[4];
    float l2 = analog[2];
    float r2 = analog[5]; 
    float target_x = analog_lx;
    float target_y = -analog_ly;
    float target_theta = analog_rx;

    if(target_x < 0.3 && target_x > -0.3)target_x = 0;
	  if(target_y < 0.3 && target_y > -0.3)target_y = 0;
	  if(target_theta < 0.2 && target_theta > -0.2)target_theta = 0;

    float factor = 2 + 2 * (r2 - l2);
    target_x *= factor;
    target_y *= factor;
    target_theta *= factor;

    
    send_data.body_speed.x = target_x;
    send_data.body_speed.y = target_y;
    send_data.body_speed.theta = target_theta;

    std::cout<<target_x<<std::endl;
    std::cout<<target_y<<std::endl;
    std::cout<<target_theta<<std::endl;

    //mekanisme dribble
    if((button_t == DRIB_SPEED_UP) && (drib_flag == 0)){
      send_data.ball_drib.drib_speed+=0.2;
      drib_flag = 1;
    }
    else if((button_t == DRIB_SPEED_DOWN) && (drib_flag == 0)){
      send_data.ball_drib.drib_speed-=0.2;
      drib_flag = 1;
    }
    else if(button_t == DRIB_OFF)send_data.ball_drib.drib_speed = 0;
    else if ((button_t != DRIB_SPEED_DOWN) && (button_t != DRIB_SPEED_UP) && (drib_flag == 1))drib_flag=0 ;

    if((button_t == LIFT_SPEED_UP) && (lift_flag == 0)){
      send_data.ball_drib.lift_speed+=1.0;
      lift_flag = 1;
    }
    else if((button_t == LIFT_SPEED_DOWN) && (lift_flag == 0)){
      send_data.ball_drib.lift_speed-=1.0;
      lift_flag = 1;
    }
    else if(button_t == LIFT_OFF)send_data.ball_drib.lift_speed = 0;
    else if((button_t != LIFT_SPEED_DOWN) && (button_t != LIFT_SPEED_UP) && (lift_flag == 1))lift_flag = 0;

    if(send_data.ball_drib.drib_speed > DRIB_MAX_SPEED)send_data.ball_drib.drib_speed = DRIB_MAX_SPEED;
    else if(send_data.ball_drib.drib_speed < 0)send_data.ball_drib.drib_speed = 0;

    // if(button_t == BTN_UP)send_data.ball_drib.lift_speed+=0.2;
    // else if(button_t == BTN_DOWN)send_data.ball_drib.lift_speed-=0.2;
    if(send_data.ball_drib.lift_speed > LIFT_MAX_SPEED)send_data.ball_drib.lift_speed = LIFT_MAX_SPEED;
    else if(send_data.ball_drib.lift_speed < 0)send_data.ball_drib.lift_speed = 0;

    std::cout<<"target drib: "<<send_data.ball_drib.drib_speed<<std::endl;
    std::cout<<"target lift: "<<send_data.ball_drib.lift_speed<<std::endl;

  joy_status = 1;
}

/**
 * Fungsi ini akan terpanggil setiap 10us, sesuai dengan settingan variabel
 * timer di constructor class ini
 */
int robotNode::udpLoop() {
  //terima
  int ret = n_udp.receive();
  // std::cout<<ret<<std::endl;
  if (ret > 0) {  // jika tidak ada atau error, ret == -1
  
    //std::cout<<"loop udp"<<std::endl;

    memcpy((uint8_t *)&recv_data, n_udp.rx_buff, sizeof(recv_from_robot));

    // send_data.body_speed.x = 1;
    // send_data.body_speed.y = 2;
    // send_data.body_speed.theta = 10;
    // std::cout<<send_data.body_speed.x<<std::endl;
    // std::cout<<send_data.body_speed.y<<std::endl;

    //kirim
    n_udp.send((uint8_t *)&send_data, sizeof(send_to_robot));

    auto odom = nav_msgs::msg::Odometry();

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_odom";
    odom.header.stamp = this->get_clock()->now();

    auto q = tf2::Quaternion();
    q.setRPY(0, 0, recv_data.g_position.theta);
    auto pose = geometry_msgs::msg::Pose();
    pose.orientation = tf2::toMsg(q);
    pose.position.x = -recv_data.g_position.x;
    pose.position.y = -recv_data.g_position.y;
    pose.position.z = 0;

    odom.pose.pose = pose;
    odom.twist.twist.linear.x = recv_data.body_speed.y;
    odom.twist.twist.linear.y = -recv_data.body_speed.x;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = recv_data.angular_speed[2];
    // odom.pose.covariance = {{
    //     1e-3, 0, 0, 0, 0, 0,
    //     0, 1e-3, 0, 0, 0, 0,
    //     0, 0, 1e6, 0, 0, 0,
    //     0, 0, 0, 1e6, 0, 0,
    //     0, 0, 0, 0, 1e6, 0,
    //     0, 0, 0, 0, 0, 1e3
    // }};
    // set covariance
    odom.pose.covariance[0] = 0.00001;
    odom.pose.covariance[7] = 0.00001;
    odom.pose.covariance[14] = 1000000000000.0;
    odom.pose.covariance[21] = 1000000000000.0;
    odom.pose.covariance[28] = 1000000000000.0;
    odom.pose.covariance[35] = 0.001;

    auto imu = sensor_msgs::msg::Imu();
    imu.header.frame_id = "imu";
    imu.header.stamp = this->get_clock()->now();
    imu.linear_acceleration.x = recv_data.linear_speed[0];
    imu.linear_acceleration.y = recv_data.linear_speed[1];
    imu.linear_acceleration.z = recv_data.linear_speed[2];
    imu.angular_velocity.x = recv_data.angular_speed[0];
    imu.angular_velocity.y = recv_data.angular_speed[1];
    imu.angular_velocity.z = recv_data.angular_speed[2];
    imu.orientation = tf2::toMsg(q);
    
    imu.orientation_covariance[0] = 0.01;
    imu.orientation_covariance[4] = 0.01;
    imu.orientation_covariance[8] = 0.01;
    imu.angular_velocity_covariance[0] = 0.01;
    imu.angular_velocity_covariance[4] = 0.01;
    imu.angular_velocity_covariance[8] = 0.01;
    imu.linear_acceleration_covariance[0] = 0.01;
    imu.linear_acceleration_covariance[4] = 0.01;
    imu.linear_acceleration_covariance[8] = 0.01;

    geometry_msgs::msg::TransformStamped t;
    t.set__header(odom.header);
    t.set__child_frame_id(odom.child_frame_id);
    t.transform.translation.x = pose.position.x;
    t.transform.translation.y = pose.position.y;
    t.transform.translation.z = 0;
    t.transform.set__rotation(tf2::toMsg(q));

    tf_broadcaster_->sendTransform(t);
    imu_pub->publish(imu);
    odometry_pub->publish(odom);
  }
  //return 0;
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topik swerve
 * vector
 */
// int robotNode::swerveSub(const robot_itsrobocon_msgs::msg::PolarVectorArray
// &msg) {
//     // msg.r.size();
//     if(msg.num == 3) {
//         send_data.swerve_speed[0].r = msg.r.at(0);
//         send_data.swerve_speed[1].r = msg.r.at(1);
//         send_data.swerve_speed[2].r = msg.r.at(2);
//         send_data.swerve_speed[0].theta = msg.theta.at(0);
//         send_data.swerve_speed[1].theta = msg.theta.at(1);
//         send_data.swerve_speed[2].theta = msg.theta.at(2);
//     }
//     return 0;
// }

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topik flag
 */
int robotNode::flagSub(const std_msgs::msg::UInt32 &msg) {
  send_data.flag = msg.data;
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topik swerve
 * vector
 */
int robotNode::speedSub(const geometry_msgs::msg::Twist &msg) {
  // need adjustment
  if (!joy_status){
      send_data.body_speed.x = -msg.linear.y;
      send_data.body_speed.y = msg.linear.x;
       send_data.body_speed.theta = -msg.angular.z;
  }
  std::cout<<msg.linear.x<<std::endl;
  std::cout<<msg.linear.y<<std::endl;
  std::cout<<msg.angular.z<<std::endl;
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topic run_status
 */
void robotNode::robot_status_callback(const std_msgs::msg::Bool &msg) {
  //@TODO: Program force stop saat msg=false
}

/**
 * Fungsi ini akan terpanggil jika ada data yang dipublish pada topic run_status
 */
// void robotNode::gripper_callback(const std_msgs::msg::Float32MultiArray &msg)
// {
//     send_data.pick_speed = msg.data.at(0);
//     send_data.conveyor_speed = msg.data.at(1);
//     send_data.gripper_pos = msg.data.at(2);
//     send_data.gripper_lift_pos = msg.data.at(3);
//     send_data.gripper_claw = (msg.data.at(4) != 0);
// }

/**
 *
 */
// void robotNode::odomResetSub(const std_msgs::msg::Bool &msg) {
//     send_data.reset_odom = (msg.data)?1:0;
// }
void robotNode::odomResetSub(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> res) {
  req.get();
  res.get();
  send_data.reset_odom = 1;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
