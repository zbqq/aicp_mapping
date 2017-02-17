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
    Eigen::Isometry3d getSensorPose(){ return world_to_last_scan; };
    Eigen::Isometry3d getBodyPose(){ return world_to_body; };
    std::vector<LidarScan>& getScans(){ return planar_scans; };
    int getId(){ return cloud_id; };
    void setId(int newId){ cloud_id = newId; };

    int getItsReferenceId(){ return its_reference_id_; };

    bool isEmpty(){ return !initialized_; };

    PM::Matrix& getPoints(){ return dp_cloud.features; };
    //PM::Matrix getColors(){ return dp_cloud.getDescriptorCopyByName("colors"); };
    //PM::Matrix getNormals(){ return dp_cloud.getDescriptorCopyByName("normals"); };

    void populateSweepScan(std::vector<LidarScan>& scans, DP& cloud, int id);
    void populateSweepScan(std::vector<LidarScan>& scans, DP& cloud, int id, int refId, bool enRef);
    void resetSweepScan();
    bool setReference()
    {
        if(enabled_reference_)
            is_reference_ = true;
        return enabled_reference_;
    };

    void addPointsToSweepScan(DP other_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPCLCloud();

  private:
    bool initialized_;
    bool is_reference_; // Is this cloud a reference cloud?
    bool enabled_reference_; // Can this cloud be used as a reference cloud?
    
    // If scan is a reading cloud this field contains the id of the reference used for alignment
    // If scan is the first reference cloud this field is set to -1 
    int its_reference_id_;

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
    Eigen::Isometry3d world_to_body;
};