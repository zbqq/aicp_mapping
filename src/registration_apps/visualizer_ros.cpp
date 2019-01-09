#include "registration_apps/visualizer_ros.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>


#include <sensor_msgs/PointCloud2.h>

using namespace std;

namespace aicp {

ROSVisualizer::ROSVisualizer(ros::NodeHandle& nh) : nh_(nh)
{
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aicp/aligned_cloud", 10);
    pose_pub_ = nh_.advertise<nav_msgs::Path>("/lidar_slam/poses",100); // The topic name may require to change
    colors_ = {
         51/255.0, 160/255.0, 44/255.0,  //0
         166/255.0, 206/255.0, 227/255.0,
         178/255.0, 223/255.0, 138/255.0,//6
         31/255.0, 120/255.0, 180/255.0,
         251/255.0, 154/255.0, 153/255.0,// 12
         227/255.0, 26/255.0, 28/255.0,
         253/255.0, 191/255.0, 111/255.0,// 18
         106/255.0, 61/255.0, 154/255.0,
         255/255.0, 127/255.0, 0/255.0, // 24
         202/255.0, 178/255.0, 214/255.0,
         1.0, 0.0, 0.0, // red // 30
         0.0, 1.0, 0.0, // green
         0.0, 0.0, 1.0, // blue// 36
         1.0, 1.0, 0.0,
         1.0, 0.0, 1.0, // 42
         0.0, 1.0, 1.0,
         0.5, 1.0, 0.0,
         1.0, 0.5, 0.0,
         0.5, 0.0, 1.0,
         1.0, 0.0, 0.5,
         0.0, 0.5, 1.0,
         0.0, 1.0, 0.5,
         1.0, 0.5, 0.5,
         0.5, 1.0, 0.5,
         0.5, 0.5, 1.0,
         0.5, 0.5, 1.0,
         0.5, 1.0, 0.5,
         0.5, 0.5, 1.0};
    //std::cout << "size of colors_ is : " << colors_.size()/3 << "\n";

}

void ROSVisualizer::publishCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                 int param, // channel name
                                 string name,
                                 int64_t utime)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *cloud_rgb);

    int secs = utime * 1E-6;
    int nsecs = (utime - (secs * 1E6)) * 1E3;

    sensor_msgs::PointCloud2 output;
    //for(auto &p: cloud_rgb->points) p.rgb = rgb;
    int nColor = cloud_rgb->size() % (colors_.size()/3);
    double r = colors_[nColor*3]*255.0;
    double g = colors_[nColor*3+1]*255.0;
    double b = colors_[nColor*3+2]*255.0;

    for (size_t i = 0; i < cloud_rgb->points.size (); i++){
        cloud_rgb->points[i].r = r;
        cloud_rgb->points[i].g = g;
        cloud_rgb->points[i].b = b;
      }

    pcl::toROSMsg(*cloud_rgb, output);

    //utime has been checked and is correct;
    //cloud is called and it is correct
    output.header.stamp = ros::Time(secs, nsecs);
    output.header.frame_id = "odom";
    cloud_pub_.publish(output);
}

void ROSVisualizer::publishPose(Eigen::Isometry3d pose, int param, string name, int64_t utime){

    nav_msgs::Path path_msg;
    posePath_.push_back(pose);
    int secs = utime*1E-6;
    int nsecs = (utime - (secs*1E6))*1E3;
    path_msg.header.stamp = ros::Time(secs, nsecs);
    path_msg.header.frame_id = "odom";

    for (size_t i = 0; i < posePath_.size(); ++i){
        geometry_msgs::PoseStamped m;
        m.header.stamp = ros::Time(secs, nsecs);
        m.header.frame_id = "odom";
        tf::poseEigenToMsg(posePath_[i], m.pose);
        path_msg.poses.push_back(m);
    }

    pose_pub_.publish(path_msg);
}

void ROSVisualizer::publishCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud,
                                 int param, // buffer size
                                 string name,
                                 int64_t utime = -1)
{
    // to be implemented
}

void ROSVisualizer::publishOctree(octomap::ColorOcTree*& octree,
                                  string channel_name)
{
    // publishOctree to be implemented.
}
}
