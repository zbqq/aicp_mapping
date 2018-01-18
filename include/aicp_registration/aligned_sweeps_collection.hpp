#include <vector>
#include <Eigen/Geometry>

#include "sweep_scan.hpp"

//#include <icp-registration/icp_utils.h>

// Point cloud class
class AlignedSweepsCollection 
{
  public:
    AlignedSweepsCollection();
    ~AlignedSweepsCollection();

    long long int getUtimeStart(){ return utime_start; };
    long long int getUtimeEnd(){ return utime_end; };
    int getNbClouds(){ return aligned_clouds.size(); };

    bool isEmpty(){ return !initialized_; };

    std::vector<SweepScan>& getClouds(){ return aligned_clouds; };
    Eigen::Matrix4f& getConstraintToReference(){ return reference_to_current_cloud; };
    SweepScan& getCurrentCloud(){ return aligned_clouds.back(); };
    SweepScan& getCloud(int index){ return aligned_clouds.at(index); };

    SweepScan& getCurrentReference(){ return aligned_clouds.at(last_reference_); };
    int getCurrentReferenceId(){ return last_reference_; };
    void updateReference(int index){ last_reference_ =  aligned_clouds.at(index).getId(); };

    void initializeCollection(SweepScan reference);
    void addSweep(SweepScan current_aligned, Eigen::Matrix4f T_ref_curr);

  private:
    bool initialized_;

    int last_reference_;

    long long int utime_start; // Time stamp of the reference cloud
    long long int utime_end; // Time stamp of last cloud aligned

    std::vector<SweepScan> aligned_clouds;  // The cloud (field dp_cloud) transformed using
                                            // reference_to_current_cloud

    Eigen::Matrix4f reference_to_current_cloud; // Current pose with respect to reference cloud
                                                  // constraint after drift correction
//    Eigen::Matrix4f previous_to_current_cloud; // Current pose with respect to previous cloud (cloud_id-1)
//                                                 // without drift correction
};
