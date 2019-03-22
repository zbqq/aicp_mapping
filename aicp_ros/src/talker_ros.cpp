#include "aicp_ros/talker_ros.hpp"
#include "aicp_utils/common.hpp"

namespace aicp {

ROSTalker::ROSTalker(ros::NodeHandle& nh, std::string fixed_frame) : nh_(nh),
                                                                     fixed_frame_(fixed_frame)
{
    footstep_plan_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/aicp/footstep_plan_request_list",10);
}

void ROSTalker::publishFootstepPlan(PathPoses& path,
                                    int64_t utime,
                                    bool reverse_path){

    if(reverse_path)
        reversePath(path);

    geometry_msgs::PoseArray path_msg;
    int secs = utime*1E-6;
    int nsecs = (utime - (secs*1E6))*1E3;
    path_msg.header.stamp = ros::Time(secs, nsecs);
    path_msg.header.frame_id = fixed_frame_;

    for (size_t i = 0; i < path.size(); ++i){
        geometry_msgs::Pose p;
        p.position.x = path[i].translation().x();
        p.position.y = path[i].translation().y();
        p.position.z = path[i].translation().z();
        Eigen::Matrix3d pose_rot = path[i].rotation();
        Eigen::Quaterniond pose_rot_q(pose_rot);
        p.orientation.x = pose_rot_q.x();
        p.orientation.y = pose_rot_q.y();
        p.orientation.z = pose_rot_q.z();
        p.orientation.w = pose_rot_q.w();

        path_msg.poses.push_back(p);
    }

    footstep_plan_pub_.publish(path_msg);
}

void ROSTalker::reversePath(PathPoses& path){

//    Eigen::Vector3d v1;
//    v1 << 1, 0, 0;
//    Eigen::Vector3d v2;
//    v2 << 1, 0, 0;

//    Eigen::Matrix3d rotx;
//    rotx = Eigen::AngleAxisd(0*M_PI/180, Eigen::Vector3d::UnitX())
//         * Eigen::AngleAxisd(0*M_PI/180, Eigen::Vector3d::UnitY())
//         * Eigen::AngleAxisd(30*M_PI/180, Eigen::Vector3d::UnitZ());
//    v2 = rotx * v2;

//    double res = angleBetweenVectors2d(v1, v2);
//    std::cout << "res: " << res << std::endl;
    std::reverse(path.begin(),path.end());

    for (size_t i = 0; i < path.size(); ++i){
        double angle = 0.0;
        if (i != path.size()-1)
        {
            Eigen::Vector2d v1;
            v1 = path[i].matrix().block<2,1>(0,0);
            Eigen::Vector2d v2;
            v2 = path[i+1].matrix().block<2,1>(0,0) - v1;
            angle = angleBetweenVectors2d(v1, v2);
        }

        // Turn about z-axis
        Eigen::Matrix3d rot;
        rot = Eigen::AngleAxisd(0*M_PI/180, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(0*M_PI/180, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(angle*M_PI/180, Eigen::Vector3d::UnitZ());

        path[i].linear() = rot * path[i].rotation();
    }
}
}
