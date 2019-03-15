#pragma once

#include "ros/node_handle.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <geometry_msgs/PoseArray.h>

namespace aicp {

class ROSTalker
{
public:
    typedef std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> PathPoses;
public:
    ROSTalker(ros::NodeHandle& nh, std::string fixed_frame);

    // Publish footstep plan
    void publishFootstepPlan(PathPoses& path,
                             int64_t utime,
                             bool reverse_path = false);
    void reversePath(PathPoses& path);

private:
    ros::NodeHandle& nh_;
    ros::Publisher footstep_plan_pub_;
    std::string fixed_frame_; // map or map_test
};
}
