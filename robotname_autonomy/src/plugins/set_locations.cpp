// Navigation Related Behaviors

#include "robotname_autonomy/plugins/set_locations.hpp"

#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// SETLOCATIONS
// Gets a list of locations from a YAML file and ensures they are not empty
SetLocations::SetLocations(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
{
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus SetLocations::tick()
{
    std::string location_file;
    const auto result = config().blackboard->get("location_file", location_file);
    if (!result) {
        std::cerr << "[" << this->name() << "] Could not read locations file from blackboard." << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        YAML::Node locations = YAML::LoadFile(location_file);
        int num_locs = locations.size();
        if (num_locs == 0) {
            std::cerr << "[" << this->name() << "] No locations found." << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        setOutput("num_locs", num_locs);
        std::cout << "[" << this->name() << "] Found " << num_locs << " locations." << std::endl;

        std::deque<std::string> location_names{};
        std::map<std::string, Pose> location_poses{};
        for (YAML::const_iterator it=locations.begin(); it!=locations.end(); ++it) {
            const auto name = it->first.as<std::string>();
            location_names.push_back(name);
            const Pose pose = it->second.as<Pose>();
            location_poses.emplace(name, pose);
        }
        // Shuffle location names to get random order in each run
        std::random_device rd;
        std::mt19937 rng(rd());
        std::shuffle(location_names.begin(), location_names.end(), rng);
        setOutput("loc_names", location_names);
        setOutput("loc_poses", location_poses);
        
    } catch (YAML::Exception const& e) {
        std::cerr << "Couldn't load locations file: " << location_file << ". Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetLocations::providedPorts()
{
    return { BT::OutputPort<int>("num_locs"),
             BT::OutputPort<std::deque<std::string>>("loc_names"),
             BT::OutputPort<std::map<std::string, Pose>>("loc_poses")
         };
}