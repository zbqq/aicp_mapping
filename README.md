# Auto-tuned ICP

Auto-tuned Iterative Closest Point \(AICP\) is a module for laser-based localization and mapping \(Nobili et al., ICRA 2017\). The implementation of AICP includes a module for localization failure prediction \(Nobili et al., ICRA 2018\).
The registration strategy is based on the *libpointmatcher* framework \(Pomerleau et al., AR 2012\).

Demonstration Video with ANYMal:

[![Watch the video](https://img.youtube.com/vi/9XMpm4VTBxU/maxresdefault.jpg)](https://youtu.be/9XMpm4VTBxU)



AICP has been tested on:

- **Carnegie Robotics Multisense SL** data from:
    - NASA Valkyrie humanoid
    - Boston Dynamics Atlas humanoid
    - IIT HyQ quadruped
    - Clearpath Husky mobile platform
- **Velodyne HDL-64** data from
    - KITTI dataset (mobile platform)
- **Velodyne VLP-16** data from
    - ANYbotics ANYmal quadruped

For inter-process communication, the package includes wrappers for:

 - [ROS](http://wiki.ros.org/ROS/Introduction)
 - [LCM](https://lcm-proj.github.io/) (discontinued)


***

### Description

The core AICP strategy is compiled in *aicp_core*.
The default steps perform frame-to-reference localization and mapping, and include:

1. On a thread, AICP accumulates laser scans, i.e. each 3D point cloud processed for registration is constituted of `batch_size` scans
2. On a second thread, it stores the first point  cloud as the **reference cloud**
3. Prior to registration, AICP computes an octree-based overlap parameter \(Nobili et al., ICRA 2018\) between the current and reference point clouds, and uses it to **auto-tune** online the outlier rejection filter of the registration strategy
4. A reference point cloud update can be trigger **either** in a windowed fashion (`reference_update_frequency`) **or** based on the risk of alignment failure prediction (`failure_prediction_mode`)
5. AICP registers  each new point cloud to the reference point cloud
6. Finally, AICP publishes a **corrected** body pose estimate at the frequency of the state estimator

#### Additional Functionalities

- **AICP localization only** -- frame-to-map
- **Go Back to Start** service (described in detail on issue https://github.com/ori-drs/aicp/issues/19#issuecomment-473247145)
    - Phase 1: SLAM
    - Phase 2: operator request and path approval
    - Phase 3: Go Back, Localization only

***

### Quick Start

#### Main dependencies

* [libpointmatcher](https://github.com/ethz-asl/libpointmatcher.git), a modular library implementing the Iterative Closest Point \(ICP\) algorithm for aligning point clouds.

* [Point Cloud Library \(PCL\)](https://github.com/pointcloudlibrary/pcl) revision pcl-1.7.0, a standalone, large scale, open project for 2D/3D image and point cloud processing.

* [Octomap](https://github.com/OctoMap/octomap.git), a probabilistic 3D mapping framework based on octrees.

* [OpenCV](https://opencv.org/) -- Open Source Computer Vision Library.

* [Eigen](https://eigen.tuxfamily.org/dox-devel/index.html), a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

#### How To Compile

1. Create folders:

```
mkdir -p  ~/aicp_base/git/ ~/aicp_base/catkin_ws/src
```

2. Set-up catkin workspace:

```
source /opt/ros/kinetic/setup.bash
cd ~/aicp_base/catkin_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

3. Clone and compile source code:

```
cd ~/aicp_base/git
git clone git@github.com:ori-drs/aicp.git
ln -s ~/aicp_base/git/* ~/aicp_base/catkin_ws/src

cd ~/aicp_base/catkin_ws
catkin build aicp_ros
```

#### How To Run

**ROS**

- **User Interface:**
    1. `roscore`
    2. EITHER `rosrun rviz rviz -d [PATH_TO]/drs_base/git/aicp/aicp_ros/config/rviz/simple_aicp.rviz`
    3. OR `roslaunch aicp_ros view_recorded_rosbag_aicp_simple.launch`
- **AICP launch:**
    - **AICP mapping -- frame-to-reference**: `roslaunch aicp_ros aicp_mapping.launch`
    - **AICP localization only -- frame-to-map**: `roslaunch aicp_ros aicp_localization_only.launch`
- **Log:**
    - `rosbag play --clock --pause yourbagfile.bag`
    - You can download the bag file from the experiment in the video here:
      - ftp://ftp.robots.ox.ac.uk/pub/outgoing/aicp/anymal_2019-04-17-12-59-42_filtered_aicp_input.bag



**LCM** (discontinued)

Help: `rosrun aicp_lcm aicp_lcm_node -h`
Example: `rosrun aicp_lcm aicp_lcm_node -s debug -b 80 -f 5`

***Note:***

* Option _"-s robot"_ is meant to be used if the corrected pose from AICP is fed back into the state estimator. Each corrected pose is published only once as soon as computed.
* Option _"-s debug"_ is meant to be used during debug. The corrected pose message is published at the same frequency as the pose estimate from the state estimator \(for visualization purposes\).


#### Misc

Required Topics (ROS):

* Point Cloud from a 3D Lidar, ideally transformed into the base frame
* Relative odometry estimate. This is the estimate of the LIDAR frame in a fixed frame. This should have low drift

To filter an ANYmal rosbag to contain only the required topics

* rosbag filter in.bag out.bag "topic=='/tf' or topic=='/state_estimator/pose_in_odom' or topic=='/point_cloud_filter/velodyne/point_cloud_filtered' or topic=='/tf_static'"


***

### Credits

```
@inproceedings{Nobili17icra,
  title = {Overlap-based {ICP} Tuning for Robust Localization of a Humanoid Robot},
  author = {S. Nobili and R. Scona and M. Caravagna and M. Fallon},
  booktitle = {{IEEE International Conference on Robotics and Automation (ICRA)}},
  location = {Singapore},
  month = {May},
  year = {2017},
}
```

```
@inproceedings{NobiliCamurri17rss,
  title = {Heterogeneous Sensor Fusion for Accurate State Estimation of Dynamic Legged Robots},
  author = {S. Nobili and M. Camurri and V. Barasuol and M. Focchi and D. Caldwell and C. Semini and M. Fallon},
  booktitle = {{Robotics: Science and Systems (RSS)}},
  location = {Boston, MA, USA},
  month = {July},
  year = {2017},
}
```

```
@inproceedings{Nobili18icra,
  title = {Predicting Alignment Risk to Prevent Localization Failure},
  author = {S. Nobili and G. Tinchev and M. Fallon},
  booktitle = {{IEEE International Conference on Robotics and Automation (ICRA)}},
  location = {Brisbane, Australia},
  month = {May},
  year = {2018},
}
```
***

### License

The License information is available in the LICENSE file contained in this project repository.

Author: Simona Nobili, March 2019.
Email: snobili@robots.ox.ac.uk, simona.nobili@ed.ac.uk

