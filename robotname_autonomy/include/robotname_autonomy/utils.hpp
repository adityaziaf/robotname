#pragma once

#include "vector"

// Struct to keep location pose data
struct Pose{
  double x;
  double y;
  double theta;
};

typedef std::vector<Pose> Poses;

// Template specialization to converts a string to Position2D.
namespace BT
{
    template <> inline Pose convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Pose output;
            output.x     = convertFromString<double>(parts[0]);
            output.y     = convertFromString<double>(parts[1]);
            output.theta = convertFromString<double>(parts[2]);
            return output;
        }
    }

    template <> inline Poses convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons

        Poses total_pose;

        auto get_poses = splitString(str,'/');

        for (auto& pose : get_poses) { 
            
            auto parts = splitString(pose, ';');
            if (parts.size() != 3)
            {
                throw RuntimeError("invalid input)");
            }
            else
            {
                Pose output;
                output.x     = convertFromString<double>(parts[0]);
                output.y     = convertFromString<double>(parts[1]);
                output.theta = convertFromString<double>(parts[2]);
                total_pose.push_back(output);
            }

        } 
        return total_pose;

    }

} // end namespace BT