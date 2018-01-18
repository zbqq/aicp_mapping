#include <vector>
#include <Eigen/Geometry>

using namespace std;

// Planar lidar scan class
class LidarScan 
{
  public:
    LidarScan(long long int t_stamp,float init_angle,float step_angle,
              std::vector< float > r,std::vector< float > i,
              Eigen::Isometry3d head_pose, Eigen::Isometry3d lidar_pose,
              Eigen::Isometry3d body_pose);
    ~LidarScan();

    long long int getUtime(){ return utime; };
    int getNbPoints(){ return ranges.size(); };
    Eigen::Isometry3d getSensorPose(){ return world_to_head * head_to_lidar; };
    Eigen::Isometry3d getBodyPose(){ return world_to_body; };

    std::vector< float > getRanges(){ return ranges; };
    std::vector< float > getIntensities(){ return intensities; };

  private:
    long long int utime;
    float rad0; // Initial angle in radians
    float radstep; // Angular resolution in radians

    std::vector< float > ranges;  // in lidar reference frame
    std::vector< float > intensities;

    Eigen::Isometry3d world_to_head;  // current head pose (w <-- h)
    Eigen::Isometry3d head_to_lidar;  // current lidar pose (h <-- l)
    Eigen::Isometry3d world_to_body;  // current head pose (w <-- h)
};