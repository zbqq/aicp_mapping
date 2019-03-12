#pragma once

#include "ros/node_handle.h"

#include <Eigen/Dense>

#include <geometry_msgs/PoseArray.h>

namespace aicp {

class ROSTalker
{
public:

    ROSTalker(ros::NodeHandle& nh, std::string fixed_frame);

    // Publish footstep plan
    void publishFootstepPlan(std::vector<Eigen::Isometry3d> &path,
                             int64_t utime);

private:
    ros::NodeHandle& nh_;
    ros::Publisher footstep_plan_pub_;
    std::string fixed_frame_; // map or map_test
};
}
