/**
 * Default: Reads raw KITTI OXTS poses and sends lcm msgs to preview poses (collections).
 * Also it loads and publishes point clouds (collections)
 * and images (if available - publish to MULTISENSE_CAMERA_LEFT).
 * Run (e.g. for drive 10):
 kitti_trajectory -d /media/snobili/SimonaHD/logs/kitti/raw/poses/10/2011_09_30_drive_0034_sync/ -e 1200
 * (writing to file option needs to be enabled using -w)
 *
 * Option 2: Reads poses from .txt file (absolute poses)
 * with [r11 r12 r13 t1 r21 r22 r23 t2 r31 r32 r33 t3] values per row.
 * where R = [r11 r12 r13
 *            r21 r22 r23
 *            r31 r32 r33]
 * and t = [t1, t2, t3]^T
 * and sends lcm msgs to preview poses (collections).
 * Also it loads and publishes point clouds (collections)
 * and images (if available - publish to MULTISENSE_CAMERA_LEFT).
 * Run (e.g. for drive 06) using DeepLO output poses:
 kitti_trajectory -d /media/snobili/SimonaHD/logs/kitti/raw/poses/06/2011_09_30_drive_0020_sync/ -e 1100
 -r /home/snobili/data/outDeepLO/out_block_3/exp_d/results/06.txt --skip_clouds 5
 * (convert .npy to .txt using python store_matrix_txt_from_npy.py)
 *
 * Option 3: Reads from any of the two above
 * and publishes poses to POSE_BODY_ALT.
 * Also it loads and publishes point clouds (collections)
 * and images (if available - publish to MULTISENSE_CAMERA_LEFT).
 * Run (e.g. for drive 06) using DeepLO output poses:
 kitti_trajectory -d /media/snobili/SimonaHD/logs/kitti/raw/poses/06/2011_09_30_drive_0020_sync/ -e 1100
 -r /home/snobili/data/outDeepLO/out_block_3/exp_d/results/06.txt --skip_clouds 5 -po POSE_BODY_ALT
 *
 * Read /media/snobili/SimonaHD/logs/kitti/raw/readme.txt for mapping odometry<->raw.
 *
 * Note: hard coded absolute paths to point clouds and images folder!
 */

// prepend with 0
#include <iomanip>

// precision
#include <limits>
typedef std::numeric_limits< double > dbl;

// args
#include <ConciseArgs>

// lcm
#include <lcmtypes/bot_core_image_t.h>
#include <lcmtypes/bot_core_pose_t.h>

#include <pronto_vis/pronto_vis.hpp> // visualize pt clds

// Eigen
#include <Eigen/Geometry>

// opencv
#include <opencv2/opencv.hpp>

Eigen::IOFormat OctaveFmt(16, 0, ", ", ";\n", "", "", "[", "]");

lcm_t* m_lcm;
pronto_vis* pc_vis_;

// Send a list of poses/objects as OBJECT_COLLECTION
int obj_coll_id = 1;
obj_cfg oconfig = obj_cfg(obj_coll_id, "Poses", 5, 1);
std::vector < pcl::PointCloud<pcl::PointXYZRGB> > pcs;

struct CommandLineConfig {
  std::string base_directory;
  std::string file_read_poses;
  bool write_poses;
  int start_frame;
  int end_frame;
  std:: string pose_output_channel;
  int skip_clouds;
  bool colour_time;
  float downsampling_resolution;
} cl_cfg;

struct Pose {
  unsigned int line_num;
  Eigen::Isometry3d transform;
};

std::vector<Pose> poses;
std::vector<std::vector<double>> oxts;

namespace fs = boost::filesystem;

void writeLineToFile(std::string out_file, Eigen::VectorXd &values)
{
  // given values (row vector) store it as a line in a file
  std::ofstream file ( out_file, std::ios_base::app | std::ios_base::out );

  if (file.is_open())
  {
    for (int i = 0; i < values.size(); i++)
    {
      file << values(i) << " ";
    }
    file << std::endl;
    file.close();
  }
  else std::cout << "[File IO] Unable to open file.\n";
  // std::cout << "[File IO] Written filename: " << out_file << std::endl;

}

void readPoses(std::string file_read_poses) {
  std::cout << "[File IO] Filename: " << file_read_poses << std::endl;
  // if (fs::is_directory(file_read_poses) && fs::is_regular_file(file_read_poses)) {
  if (fs::is_regular_file(file_read_poses)) {
    std::string filename = file_read_poses;
    std::cout << "Accessing file: " << filename << std::endl;

    std::string line;
    std::ifstream kitti_trajectories(filename);

    if (kitti_trajectories.good()) {
      unsigned int line_num = 0;
      while (std::getline(kitti_trajectories,line))
      {
        std::vector<std::string> strs;
        boost::split(strs, line, boost::is_any_of("\t "));
        // if (coords[0] != 0) {
        if(strs.at(0).compare("#") != 0) {

          Eigen::Matrix4d curr_pose;
          // curr_pose << std::stod(strs.at(0)), std::stod(strs.at(1)), std::stod(strs.at(2)), std::stod(strs.at(3)), std::stod(strs.at(4)), std::stod(strs.at(5)), std::stod(strs.at(6)), std::stod(strs.at(7)), std::stod(strs.at(8)), std::stod(strs.at(9)), std::stod(strs.at(10)), std::stod(strs.at(11)), std::stod(strs.at(12)), std::stod(strs.at(13)), std::stod(strs.at(14)), std::stod(strs.at(15));
          curr_pose << std::stod(strs.at(0)), std::stod(strs.at(1)), std::stod(strs.at(2)), std::stod(strs.at(3)), std::stod(strs.at(4)), std::stod(strs.at(5)), std::stod(strs.at(6)), std::stod(strs.at(7)), std::stod(strs.at(8)), std::stod(strs.at(9)), std::stod(strs.at(10)), std::stod(strs.at(11));

          // // directly parse the pose std::stod(strs.at(i))
          Eigen::Isometry3d curr_poseI = Eigen::Isometry3d::Identity();

          curr_poseI.translation() << curr_pose(0,3), curr_pose(1,3), curr_pose(2,3);

          Eigen::Quaterniond q = Eigen::Quaterniond(curr_pose.block<3,3>(0,0));

          curr_poseI.rotate(q);

          // // move the measurements at the height of the velodyne
          // Eigen::Isometry3d T_vel_veh = Eigen::Isometry3d::Identity();

          // T_vel_veh.translation() << 0, 0, 1.73;

          // curr_poseI = curr_poseI*T_vel_veh;

          poses.push_back({line_num, curr_poseI});

        }
        ++line_num;
      }
    }
  }
  else
    std::cout << "[KITTI] File not found." << std::endl;
}

/**
 * Reads GPS/IMU data from files to memory, requires base directory
 * (=sequence directory as parameter).
 * @param base_dir location of the oxt files
 */
void loadOxtsliteData(std::string base_dir) {
  if(fs::is_directory(base_dir+"/oxts/data")) {
    typedef std::vector<fs::path> vec;    // store paths,
    vec v;                                // so we can sort them later
    fs::path p = base_dir+"/oxts/data";
    copy(fs::directory_iterator(p), fs::directory_iterator(), back_inserter(v));
    sort(v.begin(), v.end());             // sort, since directory iteration
                                          // is not ordered on some file systems

    int fileCounter = 0;

    // for each file
    for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it) {
      if (fs::is_regular_file(*it)) {
        std::string filename = fs::canonical(*it, fs::current_path()).string();

        // std::cout << "Accessing file: " << filename << std::endl;

        std::string line;
        std::ifstream kitti_trajectories(filename);

        if (kitti_trajectories.good()) {
          while (std::getline(kitti_trajectories,line))
          {
            std::vector<std::string> strs;
            std::vector<double> coords;
            boost::split(strs, line, boost::is_any_of("\t "));
            // if (coords[0] != 0) {
            if(strs.at(0).compare("#") != 0) {
              for(size_t i = 0u; i< strs.size(); ++i) {
                coords.push_back(std::stod(strs.at(i)));
              }
              oxts.push_back(coords);
            }
          }
        }
        fileCounter++;
      }
    }
  }
}

std::vector<double> latlonToMercator(double lat, double lon, double scale) {
  std::vector<double> ret;
  double earthRadius = 6378137;
  double mx, my;
  mx = scale*lon*M_PI*earthRadius/180;
  my = scale*earthRadius*std::log(std::tan((90+lat)*M_PI/360));
  ret.push_back(mx);
  ret.push_back(my);
  return ret;
}

/**
 * Helper to compute mercator scale from latitude
 * @param  lat the first latitude measurement
 * @return     scale
 */
double latToScale(double lat) {
  double scale = cos(lat*M_PI/180.0);
  // std::cout << "Scale: " << std::setprecision(16) << scale << std::endl;
  return scale;
}

/**
 * Converts a list of oxts measurements into metric poses,
 * starting at (0,0,0) meters, OXTS coordinates are defined as
 * x = forward, y = right, z = down (see OXTS RT3000 user manual)
 * afterwards, pose{i} contains the transformation which takes a
 * 3D point in the i'th frame and projects it into the oxts
 * coordinates of the first frame.
 *
 * @param gps parsed GPS poses
 */
void convertOxtsToPose(std::vector<std::vector<double>> gps) {
  double scale = latToScale(gps[0][0]);

  Eigen::Matrix4d T_local_inv = Eigen::Matrix4d::Identity(4,4);

  std::string file_name = cl_cfg.base_directory + "oxts_poses.txt";
  if (cl_cfg.write_poses)
  {
    // Does output file exist?
    std::fstream old_file( file_name, std::ios::in );
    if (old_file)
    {
      // Yes, truncate it
      old_file.close();
      old_file.open( file_name, std::ios::out | std::ios::trunc );
    }
  }

  if (cl_cfg.end_frame == -1)
    cl_cfg.end_frame = gps.size();
  for (unsigned int i = cl_cfg.start_frame; i <= cl_cfg.end_frame; ++i) {

    // if (i == 500) {
    //   for (auto g : gps[i]) {
    //     std::cout << std::setprecision(std::numeric_limits<double>::digits10) << g << " ";
    //   }
    //   std::cout << std::endl;
    // }
    // translation
    std::vector<double> xy = latlonToMercator(gps[i][0], gps[i][1], scale);
    Eigen::Vector3d translation;
    translation << xy[0], xy[1], gps[i][2];

    // if (i == 500)
    //   std::cout << translation.transpose() << std::endl;

    // rotation
    double rx = gps[i][3], ry = gps[i][4], rz = gps[i][5]; // roll, pitch, heading
    Eigen::Matrix3d Rx, Ry, Rz, R;
    Rx << 1, 0,       0,
          0, cos(rx), -sin(rx),
          0, sin(rx), cos(rx);
    Ry << cos(ry), 0, sin(ry),
          0,       1, 0,
          -sin(ry),0, cos(ry);
    Rz << cos(rz), -sin(rz), 0,
          sin(rz), cos(rz),  0,
          0,       0,        1;

    R = Rz*Ry*Rx;

    if (T_local_inv == Eigen::Matrix4d::Identity(4,4)) {
      T_local_inv.block(0,0,3,3) = R;
      T_local_inv(0,3) = translation(0);
      T_local_inv(1,3) = translation(1);
      T_local_inv(2,3) = translation(2);
      T_local_inv(3,0) = 0;
      T_local_inv(3,1) = 0;
      T_local_inv(3,2) = 0;
      T_local_inv(3,3) = 1;
      T_local_inv = T_local_inv.inverse();
    }

    Eigen::Matrix4d homogenous_tr = Eigen::Matrix4d::Identity();
    homogenous_tr.block(0,0,3,3) = R;
    homogenous_tr(0,3) = translation(0);
    homogenous_tr(1,3) = translation(1);
    homogenous_tr(2,3) = translation(2);
    homogenous_tr(3,0) = 0;
    homogenous_tr(3,1) = 0;
    homogenous_tr(3,2) = 0;
    homogenous_tr(3,3) = 1;

    Eigen::Matrix4d pose = T_local_inv * homogenous_tr;

    Eigen::Isometry3d curr_pose = Eigen::Isometry3d::Identity();

    curr_pose.translation() << pose(0,3), pose(1,3), pose(2,3);

    // std::cout << pose(0,3) << ", " << pose(1,3) << ", " << pose(2,3) << std::endl;

    // Eigen::Quaterniond q = Eigen::Quaterniond(pose.block<3,3>(0,0));

    Eigen::Matrix3d pose_R = pose.block<3,3>(0,0);
    Eigen::Vector3d ea = pose_R.eulerAngles(2, 1, 0); // ea(0):yaw, ea(1):pitch, ea(2):roll
    Eigen::Quaterniond q;
    q =   Eigen::AngleAxisd(ea(0), Eigen::Vector3d::UnitZ()) 
        * Eigen::AngleAxisd(ea(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(ea(2), Eigen::Vector3d::UnitX());

    curr_pose.rotate(q);

    // std::cout << "i " << i << ": (degrees) " << (ea(0)*180.0)/M_PI << ", " << (ea(1)*180.0)/M_PI << ", " << (ea(2)*180.0)/M_PI << std::endl;

    // if (i >= 0 && i <= 220)
    //   std::cout << curr_pose.matrix().format(OctaveFmt) << std::endl;

    // // move the measurements at the height of the velodyne
    // Eigen::Isometry3d T_vel_veh = Eigen::Isometry3d::Identity();

    // T_vel_veh.translation() << 0, 0, 1.73;

    // curr_pose = curr_pose*T_vel_veh;

    poses.push_back({i, curr_pose});
    // std::cout << "Pose " << i+1 << ": \n" << pose_R << std::endl;
    // std::cout << "t: " << curr_pose.translation().transpose() << std::endl;

    // write pose (from GPS/IMU) to file
    if (cl_cfg.write_poses)
    {
      Eigen::VectorXd pose_to_line(12);
      pose_to_line << pose_R(0,0), pose_R(0,1), pose_R(0,2), curr_pose.translation()(0),
                      pose_R(1,0), pose_R(1,1), pose_R(1,2), curr_pose.translation()(1),
                      pose_R(2,0), pose_R(2,1), pose_R(2,2), curr_pose.translation()(2);
      writeLineToFile(file_name, pose_to_line);
    }
  }

}


void add_pose_collections(unsigned int idx_x1) {
  // std::cout << "Adding pose: " << idx_x1 << std::endl;
  Eigen::Isometry3d poseI = poses.at(idx_x1).transform;

  Isometry3dTime poseT = Isometry3dTime(idx_x1, poseI);
  oconfig.reset = 0;  // keep poses

  pc_vis_->pose_to_lcm(oconfig, poseT);
}

bot_core_pose_t getIsometry3dAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core_pose_t tf;
  tf.utime =   utime;
  tf.pos[0] = pose.translation().x();
  tf.pos[1] = pose.translation().y();
  tf.pos[2] = pose.translation().z();
  Eigen::Quaterniond r_x(pose.rotation());
  tf.orientation[0] =  r_x.w();
  tf.orientation[1] =  r_x.x();
  tf.orientation[2] =  r_x.y();
  tf.orientation[3] =  r_x.z();
  return tf;
}

void add_pose_body(unsigned int idx_x1, std::string output_channel) {
  std::cout << "Publishing pose: " << idx_x1 << std::endl;
  Eigen::Isometry3d poseI = poses.at(idx_x1).transform;

  // pc_vis_->pose_to_lcm(oconfig, poseT);
  bot_core_pose_t pose_out;

  // Publish to cl_cfg.output channel
  pose_out = getIsometry3dAsBotPose(poseI, idx_x1);
  bot_core_pose_t_publish(m_lcm, output_channel.c_str(), &pose_out);
}

void add_cloud(unsigned int idx_x1) {
  // 1. load pc
  std::stringstream cloud_loc;
  cloud_loc << cl_cfg.base_directory << "../../../../dataset/sequences/00/pcd/" <<  std::setfill('0') << std::setw(6) << idx_x1 << ".pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (cloud_loc.str(), *cloud_xyz_ptr) == -1) {
    std::cerr << std::endl << "Couldn't read file " << cloud_loc.str() << std::endl;
    // THIS IS NOT NEEDED FOR KITTI AS THERE ARE NO EMPTY POINTCLOUDS
    // push an empty pointcloud to the vector of pointclouds
    // when extracting the maps, the poses id must correspond to the cloud id
    // pcs.push_back(*cloud_ptr);
    return;
  }

  // std::cout << "Cloud XYZ ptr size: " << cloud_xyz_ptr->points.size() << std::endl;
  pcl::copyPointCloud(*cloud_xyz_ptr, *cloud_ptr);

  // std::cout << "Cloud ptr size: " << cloud_ptr->points.size() << std::endl;

  // downsample the pointcloud
  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
  voxel_grid.setInputCloud (cloud_ptr);
  voxel_grid.setLeafSize (cl_cfg.downsampling_resolution, cl_cfg.downsampling_resolution, cl_cfg.downsampling_resolution);
  voxel_grid.filter (*cloud_xyzrgb_ptr);

  // colour
  float rgb[3];
  if (cl_cfg.colour_time) {
    // normalize the utime
    float utimeN = (float) idx_x1 / (float) poses.size();
    // convert to rgb
    jet_rgb(utimeN, rgb);
    // std::cout << "Colouring by time (" << rgb[0] << ", " << rgb[1] << ", " <<rgb[2] << ")" << std::endl << std::endl << std::endl << std::endl;
  } 
  else {
    rgb[0] = (double) rand() / (RAND_MAX);
    rgb[1] = (double) rand() / (RAND_MAX);
    rgb[2] = (double) rand() / (RAND_MAX);
  }
  for (size_t i = 0u; i < cloud_xyzrgb_ptr->points.size(); ++i) {
    cloud_xyzrgb_ptr->points[i].r = rgb[0]*255;
    cloud_xyzrgb_ptr->points[i].g = rgb[1]*255;
    cloud_xyzrgb_ptr->points[i].b = rgb[2]*255;
  }

  // add to a vector of clouds
  pcs.push_back(*cloud_xyzrgb_ptr);

  // Send the collection of point cloud lists
  ptcld_cfg pconfig = ptcld_cfg(2, "Clouds", 1, 1, obj_coll_id, 0, {});

  pconfig.reset = 0;  // keep previous points
  pc_vis_->ptcld_to_lcm(pconfig, *cloud_xyzrgb_ptr, idx_x1, idx_x1);
}

void add_left_image(int idx_x1) {

  std::stringstream img_loc;
  img_loc << "/media/snobili/SimonaHD/logs/kitti/dataset/sequences/00/image_2/" <<  std::setfill('0') << std::setw(6) << idx_x1 << ".png";
  // img_loc << "/home/snobili/logs/kitti/dataset/sequences/06/image_2/" <<  std::setfill('0') << std::setw(6) << idx_x1 << ".png";
  // img_loc << "/home/snobili/data/optical-flow-from-FlowNetS/out_rgb/out-" << idx_x1 << ".png";

  cv::Mat img;
  img = cv::imread(img_loc.str(), CV_LOAD_IMAGE_COLOR);   // Read the file

  if (!img.data )                              // Check for invalid input
  {
    std::cout <<  "Could not open or find image " << img_loc.str() << std::endl ;
    return;
  }

  int n_colors = 3;
  bot_core_image_t image_left;
  image_left.utime = 0;
  image_left.width = img.cols;
  image_left.height= img.rows;
  image_left.row_stride =n_colors*img.cols;
  image_left.pixelformat =BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB;
  image_left.size =n_colors*img.cols*img.rows;
  image_left.data = img.data;
  image_left.nmetadata =0;
  image_left.metadata = NULL;


  // 2. Publish the filtered cloud
  bot_core_image_t_publish(m_lcm, "MULTISENSE_CAMERA_LEFT", &image_left);
}

void publish(std::string pose_output_channel) {
  // first add a pose
  for (size_t i = 0u; i < poses.size(); ++i) {
    if (!pose_output_channel.empty())
      add_pose_body(i, pose_output_channel);
    add_pose_collections(i);
    if (i % cl_cfg.skip_clouds == 0)
      add_cloud(i);
    add_left_image(i);
    boost::this_thread::sleep(boost::posix_time::milliseconds(250)); //seconds (publish at ~5Hz)
  }
}

int main(int argc, char **argv) {
  cl_cfg.base_directory = ""; // where're the clouds at?
  cl_cfg.file_read_poses = ""; // path to .txt file
  cl_cfg.write_poses = false;
  cl_cfg.start_frame = 0;
  cl_cfg.end_frame = -1;
  cl_cfg.pose_output_channel = "";
  cl_cfg.skip_clouds = 20;
  cl_cfg.downsampling_resolution = 0.1;
  cl_cfg.colour_time = true;

  ConciseArgs parser(argc, argv, "");
  parser.add(cl_cfg.base_directory, "d", "base_directory", "Absolute path to sequence directory (ends with _sync)");
  parser.add(cl_cfg.file_read_poses, "r", "file_read_poses", "Absolute path to poses directory (poses in matrix form), from sync folder otherwise");
  parser.add(cl_cfg.write_poses, "w", "write_poses", "boolean should I (over)write the GPS poses to oxts_poses.txt");
  parser.add(cl_cfg.start_frame, "s", "start_frame", "Start frame (should be >= 0)");
  parser.add(cl_cfg.end_frame, "e", "end_frame", "End frame (should be <= sequence size)");
  parser.add(cl_cfg.pose_output_channel, "po", "pose_output_channel", "Poses output channel, only to collections otherwise");
  parser.add(cl_cfg.skip_clouds, "sc", "skip_clouds", "Publish one cloud every skip_clouds");
  parser.add(cl_cfg.downsampling_resolution, "dr", "downsampling_resolution", "Resolution for clouds downsampling");
  parser.add(cl_cfg.colour_time, "ct", "colour_time", "boolean should I colour by time");
  parser.parse();

  std::cout << "============================" << std::endl
            << "Parsed Command Line Config"   << std::endl
            << "============================" << std::endl;

  std::cout << "Base directory: "           << cl_cfg.base_directory                << std::endl;
  std::cout << "Read GPS poses: "           << cl_cfg.file_read_poses               << std::endl;
  std::cout << "Write GPS poses: "          << cl_cfg.write_poses                   << std::endl;
  std::cout << "Start frame: "              << cl_cfg.start_frame                   << std::endl;
  std::cout << "End frame: "                << cl_cfg.end_frame                     << std::endl;
  std::cout << "Publish poses to: "         << cl_cfg.pose_output_channel           << std::endl;
  std::cout << "Skip clouds: "              << cl_cfg.skip_clouds                   << std::endl;
  std::cout << "Downsampling resolution: "  << cl_cfg.downsampling_resolution       << std::endl;
  std::cout << "Colour by time: "           << cl_cfg.colour_time                   << std::endl;

  m_lcm = lcm_create(NULL);
  pc_vis_ = new pronto_vis( m_lcm );

  std::cout << "[KITTI] Starting kitti_construct_map." << std::endl;
  if (cl_cfg.file_read_poses.empty()) {
    std::cout << "[KITTI] Parsing GPS coordinates." << std::endl;
    loadOxtsliteData(cl_cfg.base_directory);
    std::cout << "[KITTI] Finished parsing. Total files read: " << oxts.size() << std::endl;
    std::cout << "[KITTI] Converting GPS coordinates to poses." << std::endl;
    convertOxtsToPose(oxts);
  }
  else {
    std::cout << "[KITTI] Reading poses." << std::endl;
    readPoses(cl_cfg.file_read_poses);
  }

  std::cout << "[KITTI] Publishing clouds to poses." << std::endl;

  publish(cl_cfg.pose_output_channel);

  return 0;
}

