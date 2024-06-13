#include "rclcpp/rclcpp.hpp"
#include "robotname_msgs/srv/get_best_silo.hpp"
#include "robotname_msgs/msg/detect_silo_array.hpp"

#include <memory>

using namespace std::placeholders;

class GetBestSilo : public rclcpp::Node {
  public:
    GetBestSilo() : Node("GetBestSilo")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

      _service = this->create_service<robotname_msgs::srv::GetBestSilo>("get_best_silo",
        std::bind(&GetBestSilo::handle_service, this, _1, _2),
        qos_profile);
      _subscriber = this->create_subscription<robotname_msgs::msg::DetectSiloArray>
        ("/silo/objects/raw", rclcpp::QoS(10),
            std::bind(&GetBestSilo::detection_callback, this, _1));
    }
     ;

    void handle_service(const std::shared_ptr<robotname_msgs::srv::GetBestSilo::Request> request, 
    const std::shared_ptr<robotname_msgs::srv::GetBestSilo::Response> response)
    {
      
  
      if(_last_msg)
      {
        int i = 1, index = -1;
        int max_score = -10; // Initialize with a low score
        for(auto & object : _last_msg->detections)
        {
            std::string team_color, enemy_color;
            if(request->team_color == "red"){
              team_color = "red";
              enemy_color = "blue";
            }else if(request->team_color == "blue"){
              team_color = "blue";
              enemy_color = "red";
            }else{
              RCLCPP_INFO(this->get_logger(), "Invalid Team Color");
              response->status=false;
            }
            int score = silo_score(object.ball, team_color, enemy_color, request->mode);
            std::cout<<i<<" score: "<<score<<std::endl;
            if (score > max_score) {
                index = i;
                max_score = score;
            }
            i++;
            // object.ball.clear
        }
        response->silo=index;
        std::cout<<"Response: "<<response->silo<<std::endl;
        response->status=true;
        return;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Empty Topic Value");
      }
      response->status=false;
    }

    void detection_callback(const robotname_msgs::msg::DetectSiloArray::SharedPtr msg)
    {
      _last_msg = msg;
    }

  private:
    int silo_score(std::vector<std::string> ball_in_silo, std::string team_color, std::string enemy_color, int mode) {
        // Check for null values
        bool has_null = false;
        for (int i = 0; i < 3; i++) {
            if (ball_in_silo[i] == "null") {
                has_null = true;
            }
        }
        if (!has_null) {
            return -2; // Return -2 if there are no "null" values
        }

        // Array of combinations
        std::string arr_comb[7][3];
        if (mode == 1) {
            std::string temp_arr[7][3] = {
                {enemy_color, "null", "null"},
                {team_color, "null", "null"},
                {"null", "null", "null"},
                {enemy_color, enemy_color, "null"},
                {team_color, team_color, "null"},
                {enemy_color, team_color, "null"},
                {team_color, enemy_color, "null"}
            };

            for (int i = 0; i < 7; ++i) {
                for (int j = 0; j < 3; ++j) {
                    arr_comb[i][j] = temp_arr[i][j];
                }
            }
        }else{
          RCLCPP_INFO(this->get_logger(), "Invalid Mode");
          return -1;
        }

        // Check for matching combination
        for (int i = 0; i < 7; i++) {
            bool match = true;
            for (int j = 0; j < 3; j++) {
                if (ball_in_silo[j] != arr_comb[i][j]) {
                    match = false;
                    break;
                }
            }
            if (match) {
                return i; // Return the index of the matching combination
            }
        }
    
    return -1; // Return -1 if no match is found
}
    rclcpp::Service<robotname_msgs::srv::GetBestSilo>::SharedPtr _service;
    rclcpp::Subscription<robotname_msgs::msg::DetectSiloArray>::SharedPtr _subscriber;
    robotname_msgs::msg::DetectSiloArray::SharedPtr _last_msg;

};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<GetBestSilo>node = std::make_shared<GetBestSilo>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Get Best Silo Server Ready");

  rclcpp::spin(node);
  rclcpp::shutdown();
}