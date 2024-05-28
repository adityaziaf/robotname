#include <functional>
#include <memory>
#include <thread>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotname_msgs/action/setup_menu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include "tf2/utils.h"

#include "behaviortree_ros2/bt_action_node.hpp"

class SetupMenuServer : public rclcpp::Node
{
public:
  using SetupMenu = robotname_msgs::action::SetupMenu;
  using GoalHandleSetupMenu = rclcpp_action::ServerGoalHandle<SetupMenu>;

  explicit SetupMenuServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("SetupMenu_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<SetupMenu>(
      this,
      "setup_menu",
      std::bind(&SetupMenuServer::handle_goal, this, _1, _2),
      std::bind(&SetupMenuServer::handle_cancel, this, _1),
      std::bind(&SetupMenuServer::handle_accepted, this, _1));
    // 

    initpose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",1);
  }

private:
    int start_ = 0;
    int silo2_ = 3;
    int silo3_ = 3;
    int silo4_ = 3;

  rclcpp_action::Server<SetupMenu>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_pub;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const SetupMenu::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSetupMenu> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSetupMenu> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&SetupMenuServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSetupMenu> goal_handle)
  {
    //RCLCPP_INFO(this->get_logger(), "Executing goal");
    //rclcpp::Rate loop_rate(100);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SetupMenu::Feedback>();
    auto result = std::make_shared<SetupMenu::Result>();

    int choice;
    bool send;
    start_ = 1;
    
    do {
        // Display the menu
        send = false;
        std::string input;
        // std::cout << "Menu:\n";
        // std::cout << "1. Start from\n";
        // std::cout << "2. Silo2\n";
        // std::cout << "3. Silo3\n";
        // std::cout << "4. Silo4\n";
        // std::cout << "5. Confirm\n";
        // std::cout << "Enter your choice: ";
        // std::cin >> choice;

        std::cout<<"===INPUT===\n";
        std::cin >> input;
         // Check if input is valid
        // if (std::cin.fail()) {
        //     std::cout << "Invalid input. Please enter a number.\n";
        //     std::cin.clear(); // Clear error flag
        //     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
        //     continue; // Skip to next iteration of the loop
        // }

        // int input;
        
        if (input == "0")silo2_ = 0;
        else if(input == "1")silo2_ = 1; 
        else if(input == "4")silo2_ = 2; 
        else if(input == "7")silo2_ = 3; 
        else if(input == "000")silo3_ = 0; 
        else if(input == "2")silo3_ = 1; 
        else if(input == "5")silo3_ = 2; 
        else if(input == "8")silo3_ = 3; 
        else if(input == ".")silo4_ = 0; 
        else if(input == "3")silo4_ = 1; 
        else if(input == "6")silo4_ = 2; 
        else if(input == "9")silo4_ = 3; 
        else if(input == "+")start_ = 0; 
        else if(input == "-")start_ = 1; 
        else if(input == "*")send = true;


        // else if (input == "000"){
        //    = ',';
        // }
        // // Handle user choice
        // switch (input) {
        //     case '0':
        //       silo2_ = 0;
        //       break;
        //     case '1':
    
        //         // std::cout << "Start From:";
        //         // std::cin >> input;
        //         // start_ = input;
        //         silo2_ = 1;
        //         break;
        //     case '4':
               
        //         // std::cout << "Silo2:";
        //         // std::cin >> input;
        //         // silo2_ = input;
        //         silo2_ = 2;
        //         break;
        //     case '7':
              
        //         // std::cout << "Silo3:";
        //         // std::cin >> input;
        //         silo2_ = 3;
        //         break;
        //     case '2':
                
        //         // std::cout << "Silo4:";
        //         // std::cin >> input;
        //         silo3_ = 1;
        //         break;

        //     case '5':
        //         // std::cout << "Confirmation received. Exiting...\n";
        //         silo3_ = 2;
        //         break;

        //     case '8':
        //         // std::cout << "Confirmation received. Exiting...\n";
        //         silo3_ = 3;
        //         break;

        //     case '3':
        //         // std::cout << "Confirmation received. Exiting...\n";
        //         silo4_ = 1;
        //         break;
        //     case '6':
        //         // std::cout << "Confirmation received. Exiting...\n";
        //         silo4_ = 2;
        //         break;
        //     case '9':
        //         // std::cout << "Confirmation received. Exiting...\n";
        //         silo4_ = 3;
        //         break;
        //     case '+':

        //         // std::cout << "Confirmation received. Exiting...\n";43691

        //     default:
        //         std::cout << "Invalid choice. Please try again.\n";
        //         break;
        // }
      
    } while (!send);

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->get_clock()->now();

        tf2::Quaternion q;

        q.setRPY(0, 0, -3.14);
        q.normalize();
        msg.pose.pose.orientation = tf2::toMsg(q);
        msg.pose.pose.position.x  = 0.0;

        if(start_ == 1)
        {
          msg.pose.pose.position.x  = 5.0;
        }

        
        initpose_pub->publish(msg);
        
    if (rclcpp::ok())
    {
      result->silo2 = silo2_;
      result->silo3 = silo3_;
      result->silo4 = silo4_;

      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

};  // class SetupMenuServer


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetupMenuServer>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setup Menu Server Ready");

  rclcpp::spin(node);

  return 0;
}
