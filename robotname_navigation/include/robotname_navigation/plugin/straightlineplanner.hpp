#include "robotname_navigation/baseclass/planner.hpp"

class straightLinePlanner : public planner
{
    public:
    void initialize() override;
    nav_msgs::msg::Path createPlan(geometry_msgs::msg::PoseStamped start, geometry_msgs::msg::PoseStamped goal) override;

    private:
    rclcpp::Node::SharedPtr node_; 
};