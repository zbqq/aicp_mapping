#pragma once

#include <lcmtypes/bot_core/double_array_t.hpp>
#include <lcmtypes/bot_core/planar_lidar_t.hpp>
#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/bot_core/rigid_transform_t.hpp>

#include "cloud_accumulate/cloud_accumulate.hpp"

#include "aicp_registration/app.hpp"
#include "aicp_registration/registration.hpp"
#include "aicp_overlap/overlap.hpp"
#include "aicp_classification/classification.hpp"

#include "aicp_lcm/visualizer_lcm.hpp"

namespace aicp {

class AppLCM : public App {
public:
    AppLCM(boost::shared_ptr<lcm::LCM> &lcm,
           const CommandLineConfig& cl_cfg,
           CloudAccumulateConfig ca_cfg,
           RegistrationParams reg_params,
           OverlapParams overlap_params,
           ClassificationParams class_params);

    void planarLidarHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& channel,
                            const bot_core::planar_lidar_t* msg);

    void robotPoseHandler(const lcm::ReceiveBuffer* rbuf,
                          const std::string& channel,
                          const  bot_core::pose_t* msg);

    // Tool functions
    bot_core::pose_t getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime);
    Eigen::Isometry3d getPoseAsIsometry3d(const bot_core::pose_t* pose);
    Eigen::Isometry3d getBodyAsIsometry3d(const bot_core::rigid_transform_t* pose);
    Eigen::Isometry3d getTransfParamAsIsometry3d(PM::TransformationParameters T);

private:
    boost::shared_ptr<lcm::LCM> lcm_;
    // Used for: convertCloudProntoToPcl
    pronto_vis* pc_vis_;
    CloudAccumulate* accu_;
    CloudAccumulateConfig ca_cfg_;
    BotParam* botparam_;
    BotFrames* botframes_;

    int getTransWithMicroTime(BotFrames *bot_frames,
                             const char *from_frame,
                             const char *to_frame,
                             const int64_t& utime,
                             Eigen::Isometry3d & mat);

};
}
