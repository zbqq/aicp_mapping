#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include <ConciseArgs>


#include <pronto_vis/pronto_vis.hpp> // visualize pt clds
#include <pcl/io/pcd_io.h>

float packColor(unsigned char* color) {
    return color[0] + color[1] * 256.0 + color[2] * 256.0 * 256.0;
}

int
main (int argc, char** argv)
{
  std::string pcd_file = "pcd_filename";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(pcd_file, "v", "pcd_file","pcd_file");
  opt.parse();

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good()){
    std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  }


  pronto_vis* pc_vis_;
  pc_vis_ = new pronto_vis( lcm->getUnderlyingLCM() );
  int reset =1;
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1,"Pose 1",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2,"Point Cloud 1",1,1, 1,0,{0.0,0.0,0.0})); // use the RGB values in the point cloud
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3,"Pose 2",5,reset) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(4,"Point Cloud 2",1,1, 3,1,{1.0,0.0,0.0})); // use the user RGB
  pc_vis_->obj_cfg_list.push_back( obj_cfg(5,"Pose 3",5,reset) );

  // 1a. Create a pose:
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  int64_t utime = 1;
  Isometry3dTime poseT = Isometry3dTime(utime, pose);
  pc_vis_->pose_to_lcm_from_list(1, poseT);

  // 1b. Send a point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd_file, *cloud) == -1){
    std::cout << "Couldn't read file "<< pcd_file << "\n";
    return (-1);
  }
  std::cout << "Loaded " << cloud->width * cloud->height << " data points from file\n";
  pc_vis_->ptcld_to_lcm_from_list(2, *cloud, utime, utime);

  // 2a. Translate the pose and send the point cloud and the pose again - translated by this offset
  poseT.pose.translation() = Eigen::Vector3d(0,2,0);
  pc_vis_->pose_to_lcm_from_list(3, poseT);
  pc_vis_->ptcld_to_lcm_from_list(4, *cloud, utime, utime);


  // 3. Create a vector of poses and publish them all
  std::vector<Isometry3dTime> posesT;
  Isometry3dTime poseT2 =  Isometry3dTime(0, Eigen::Isometry3d::Identity()) ;
  poseT2.pose.translation() = Eigen::Vector3d(-4,2,0);   poseT2.utime = 1; posesT.push_back(poseT2);
  poseT2.pose.translation() = Eigen::Vector3d(-4,2.1,0); poseT2.utime = 2; posesT.push_back(poseT2);
  poseT2.pose.translation() = Eigen::Vector3d(-4,2.2,0); poseT2.utime = 3; posesT.push_back(poseT2);
  poseT2.pose.translation() = Eigen::Vector3d(-4,2.3,0); poseT2.utime = 4; posesT.push_back(poseT2);
  poseT2.pose.translation() = Eigen::Vector3d(-4,2.4,0); poseT2.utime = 5; posesT.push_back(poseT2);
  poseT2.pose.translation() = Eigen::Vector3d(-4,2.5,0); poseT2.utime = 6; posesT.push_back(poseT2);
  poseT2.pose.translation() = Eigen::Vector3d(-4,2.6,0); poseT2.utime = 7; posesT.push_back(poseT2);
  poseT2.pose.translation() = Eigen::Vector3d(-4,2.7,0); poseT2.utime = 8; posesT.push_back(poseT2);
  poseT2.pose.translation() = Eigen::Vector3d(-4,2.8,0); poseT2.utime = 9; posesT.push_back(poseT2);
  pc_vis_->pose_collection_to_lcm_from_list(5, posesT);


  // 4. Move a pose which will indirectly move the associated point cloud, without republishing it 
  std::cout << "wait 5 seconds, then translate the red point cloud by republishing the pose its attached to. The original point cloud is not retransmitted\n";
  sleep(5);
  poseT.pose.translation() = Eigen::Vector3d(0,4,0); // nominal head height
  pc_vis_->pose_to_lcm_from_list(3, poseT);


  return (0);
}

