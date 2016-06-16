#include <vector>
#include <Eigen/Geometry>

#include "lidar_scan.hpp"

#include <icp-registration/icp_utils.h>

// Point cloud class
class SweepScan 
{
  public:
    SweepScan();
    ~SweepScan();

    long long int getUtimeStart(){ return utime_start; };
    long long int getUtimeEnd(){ return utime_end; };
    int getNbPoints(){ return dp_cloud.getNbPoints(); };

    DP& getCloud(){ return dp_cloud; };
    std::vector<LidarScan>& getScans(){ return planar_scans; };
    int getId(){ return cloud_id; };

    bool isEmpty(){ return !initialized_; };

    PM::Matrix& getPoints(){ return dp_cloud.features; };
    //PM::Matrix getColors(){ return dp_cloud.getDescriptorCopyByName("colors"); };
    //PM::Matrix getNormals(){ return dp_cloud.getDescriptorCopyByName("normals"); };

    void populateSweepScan(std::vector<LidarScan>& scans, DP& cloud, int id);
    void resetSweepScan();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPCLCloud();

  private:
    bool initialized_;

    int cloud_id;
    long long int utime_start; // Time stamp of first planar scan
    long long int utime_end; // Time stamp of last planar scan 
                             // (it represent the sweep utime)

    std::vector<LidarScan> planar_scans;
    DP dp_cloud; // Point cloud representation (type from pointmatcher lib). 
                 // Descriptors: colors, normals
                 // Reference frame: local 

    Eigen::Isometry3d relative_motion; // first_to_last_scan used to accumulate cloud
                                       // without drift correction
    Eigen::Isometry3d world_to_last_scan;  // world_to_current_cloud = world_to_last_scan
};