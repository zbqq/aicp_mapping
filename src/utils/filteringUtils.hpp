#ifndef SRC_DRAWINGUTIL_HPP_
#define SRC_DRAWINGUTIL_HPP_

#include <Eigen/Dense>

#include <icp-registration/icp_utils.h>

void overlapFilter(Eigen::Isometry3d world_to_body_1, Eigen::Isometry3d world_to_body_2, 
                   DP &first_cloud, DP &second_cloud);

#endif
