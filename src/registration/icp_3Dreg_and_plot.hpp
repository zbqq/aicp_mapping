#ifndef ICP_3DREG_AND_PLOT_HPP_
#define ICP_3DREG_AND_PLOT_HPP_

#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include "boost/filesystem.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/pointcloud2_t.hpp>
#include <pronto_utils/conversions_lcm.hpp>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>

#include <icp-registration/vtkUtils.h>
#include <icp-registration/icp_utils.h>
#include <icp-registration/cloud_accumulate.hpp>

class RegistrationConfig
{
  public:
    RegistrationConfig();

    const char *homedir;

    string cloud_name_A;
    string cloud_name_B;

    string configFile3D_;
    string initTrans_; //initial transformation for the reading cloud in the form [x,y,theta]

  private:
};

class Registration{
  public:
    Registration(boost::shared_ptr<lcm::LCM> &lcm_, const RegistrationConfig& reg_cfg_);
    
    ~Registration(){
    }   

    PM::ICP getIcp(){ return icp_; } 
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(int idx){ 
      if (idx == 0) 
        return reference_cloud_; 
      else if (idx == 1)
        return transformed_input_cloud_;
      else if (idx == 2)
        return input_cloud_;
      else //idx == 3
        return initialized_input_cloud_; }
    
    PM::TransformationParameters getTransform(){ return out_transform_; }
    DP getDataOut(){ return out_cloud_; }
    
    void publishCloud(pronto_vis* vis, int cloud_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void getICPTransform(DP &cloud_in, DP &cloud_ref);

    void setDefaultIcp(){ icp_.setDefault(); }

    void setConfigFile(string configName){ 
        reg_cfg_.configFile3D_.clear();
        reg_cfg_.configFile3D_.append(configName); }

  private:
    boost::shared_ptr<lcm::LCM> lcm_;
    RegistrationConfig reg_cfg_;

    // Create the default ICP algorithm
    PM::ICP icp_;
  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr initialized_input_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_input_cloud_;
    PM::TransformationParameters out_transform_;    
    DP out_cloud_; 
};

#endif
