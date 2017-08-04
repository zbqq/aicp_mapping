#include "octrees_overlap.hpp"

namespace aicp {

  OctreesOverlap::OctreesOverlap(const OverlapParams& params) :
    params_(params)
  {
    tree_  = new ColorOcTree(params_.octree_based.octomapResolution);

    overlap_ = -1.0;

    // define colors
    yellow = new ColorOcTreeNode::Color(250,250,0);
    green = new ColorOcTreeNode::Color(0,102,0);
    blue = new ColorOcTreeNode::Color(0,0,255);
    red = new ColorOcTreeNode::Color(255,0,0);
  }

  OctreesOverlap::~OctreesOverlap() {}

  void OctreesOverlap::setReferenceTree(pcl::PointCloud<pcl::PointXYZ> &ref_cloud, Eigen::Isometry3d ref_pose)
  {
    tree_ = new ColorOcTree(params_.octree_based.octomapResolution);

    // Setting reference tree
    createTree(ref_cloud, ref_pose, tree_, yellow);
  }

  ColorOcTree* OctreesOverlap::computeOverlap(pcl::PointCloud<pcl::PointXYZ> &ref_cloud, pcl::PointCloud<pcl::PointXYZ> &read_cloud,
                                              Eigen::Isometry3d ref_pose, Eigen::Isometry3d read_pose,
                                              ColorOcTree* reading_tree){
    // Create octree from reference cloud (wrt robot point of view)
    setReferenceTree(ref_cloud, ref_pose);

    // Create reading tree (if not filled yet)
    if(reading_tree->size() == 0)
      createTree(read_cloud, read_pose, reading_tree, green);

    // count tree nodes and color overlapping nodes
    int count_nodes_read = 0;
    int count_nodes_ref = 0;
    int overlapping_nodes = 0;
    getOverlappingNodes(tree_, reading_tree, overlapping_nodes, count_nodes_ref, count_nodes_read);

    // Compute trees overlap
    float treeAoverlap, treeBoverlap;
    treeAoverlap = float(overlapping_nodes) / float(count_nodes_ref);
    treeBoverlap = float(overlapping_nodes) / float(count_nodes_read);

    // Set overlap params
    float minOverlap = min(treeAoverlap,treeBoverlap);
    //float maxOverlap = max(treeAoverlap,treeBoverlap);
    overlap_ = minOverlap * 100.0;
    //overlap_[1] = (minOverlap / maxOverlap) * 100.0;
    //overlap_[1] = abs(maxOverlap - minOverlap) * 100.0;
    //overlap_[2] = treeAoverlap * 100.0;
    //overlap_[3] = treeBoverlap * 100.0;
    //overlap_[4] = overlapping_nodes;
    //overlap_[5] = count_nodes_ref;
    //overlap_[6] = count_nodes_read;

    /*cout << "----------[Octrees Overlap] Statistics-----------" << endl;
    cout << "# of overlapping nodes: " << overlapping_nodes << endl;
    cout << "# of nodes treeA count: " << count_nodes_ref << endl;
    cout << "# of nodes treeB count: " << count_nodes_read << endl;
    cout << "treeAoverlapPerc: " << treeAoverlap * 100.0 << " %" << endl;
    cout << "treeBoverlapPerc: " << treeBoverlap * 100.0 << " %" << endl;
    cout << "octreesCombinedOverlap: " << overlap_[1] << " %" << endl;
    cout << "---------------------------------------------------" << endl;*/

    return tree_;
  }

  // FUNCTION TO BE TESTED ------------------------------
  float OctreesOverlap::computeLoopClosureFromOverlap(ColorOcTree* treeA, ColorOcTree* treeB)
  {
    // count tree nodes and color overlapping nodes
    int count_nodes_read = 0;
    int count_nodes_ref = 0;
    int overlapping_nodes = 0;
    getOverlappingNodes(treeA, treeB, overlapping_nodes, count_nodes_ref, count_nodes_read);

    // Compute trees overlap
    std::vector<float> loop_closure_overlap = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
    float treeAoverlap, treeBoverlap;
    treeAoverlap = float(overlapping_nodes) / float(count_nodes_ref);
    treeBoverlap = float(overlapping_nodes) / float(count_nodes_read);

    // TEMPORARY: PARAMS USED TO DEBUG
    // Set overlap params
    float minOverlap = min(treeAoverlap,treeBoverlap);
    float maxOverlap = max(treeAoverlap,treeBoverlap);
    loop_closure_overlap[0] = minOverlap * 100.0;
    loop_closure_overlap[1] = abs(maxOverlap - minOverlap) * 100.0;
    loop_closure_overlap[2] = treeAoverlap * 100.0;
    loop_closure_overlap[3] = treeBoverlap * 100.0;
    loop_closure_overlap[4] = overlapping_nodes;
    loop_closure_overlap[5] = count_nodes_ref;
    loop_closure_overlap[6] = count_nodes_read;

    cout << "-------[Octrees Overlap] LOOP CLOSURE Statistics-------" << endl;
    cout << "# of overlapping nodes: " << overlapping_nodes << endl;
    cout << "# of nodes treeA count: " << count_nodes_ref << endl;
    cout << "# of nodes treeB count: " << count_nodes_read << endl;
    cout << "treeAoverlapPerc: " << treeAoverlap * 100.0 << " %" << endl;
    cout << "treeBoverlapPerc: " << treeBoverlap * 100.0 << " %" << endl;
    cout << "octreesCombinedOverlap: " << loop_closure_overlap[1] << " %" << endl;
    cout << "-------------------------------------------------------" << endl;

    return loop_closure_overlap[0];
  } // FUNCTION TO BE TESTED ------------------------------

  void OctreesOverlap::getOverlappingNodes(ColorOcTree* treeA, ColorOcTree* treeB, int& overlapping_nodes, int& count_nodes_ref, int& count_nodes_read)
  {
    treeA->expand();
    treeB->expand();

    for(ColorOcTree::tree_iterator it=treeB->begin_tree(),
        end=treeB->end_tree(); it!= end; ++it) {
      if (it.isLeaf())
      {
        ColorOcTreeNode* node_read = treeB->search(it.getKey());
        if(node_read != NULL)
        {
          count_nodes_read += 1;
        }
      }
    }

    for(ColorOcTree::tree_iterator it=treeA->begin_tree(),
        end=treeA->end_tree(); it!= end; ++it) {
      if (it.isLeaf())
      {
        ColorOcTreeNode* node_ref = treeA->search(it.getKey());
        if(node_ref != NULL)
        {
          count_nodes_ref += 1;
        }
        ColorOcTreeNode* node_read = treeB->search(it.getKey());
        if(node_read != NULL)
        {
          overlapping_nodes += 1;

          node_read->setColor(*blue); // set color to blue
        }
      }
    }

    treeA->prune();
    treeB->prune();
  }

  void OctreesOverlap::createTree(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Isometry3d pose, ColorOcTree* output_tree, ColorOcTreeNode::Color* color)
  {
    output_tree->clear();
    double maxrange = -1;
    int max_scan_no = -1;
    unsigned char compression = 0;

    // 1. Convert to octomap graph:
    ScanGraph* graph = convertPointCloudToScanGraph(cloud, pose);

    timeval start;
    timeval stop;

    gettimeofday(&start, NULL);  // start timer
    unsigned int numScans = graph->size();
    unsigned int currentScan = 1;
    bool discretize = false;

    // get default sensor model values:
    ColorOcTree emptyTree(0.1);
    double clampingMin = emptyTree.getClampingThresMin();
    double clampingMax = emptyTree.getClampingThresMax();
    double probMiss = emptyTree.getProbMiss();
    double probHit = emptyTree.getProbHit();

    output_tree->setClampingThresMin(clampingMin);
    output_tree->setClampingThresMax(clampingMax);
    output_tree->setProbHit(probHit);
    output_tree->setProbMiss(probMiss);

    for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {
      output_tree->insertPointCloud((*scan_it)->scan, (*scan_it)->pose.trans(), maxrange, false, discretize);

      if (compression == 2){
        output_tree->toMaxLikelihood();
        output_tree->prune();
      }

      if ((max_scan_no > 0) && (currentScan == (unsigned int) max_scan_no))
        break;

      currentScan++;
    }

    gettimeofday(&stop, NULL);  // stop timer
    double time_to_insert = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
    cout << "[Octrees Overlap] Time to create octree: " << time_to_insert << " seconds." << endl;

    // get rid of graph in mem before doing anything fancy with tree (=> memory)
    delete graph;

    // Visualization: Color octree
    output_tree->expand();
    for(ColorOcTree::tree_iterator it=output_tree->begin_tree(),
        end=output_tree->end_tree(); it!= end; ++it) {
      if (it.isLeaf()) {
        point3d coord = output_tree->keyToCoord(it.getKey());
        // Set nodes color
        ColorOcTreeNode* n = output_tree->setNodeValue(coord, true);
        n = output_tree->updateNode(coord, true);
        n->setColor(*color);
      }
    }
    output_tree->updateInnerOccupancy();
    output_tree->prune();
  }

  ScanGraph* OctreesOverlap::convertPointCloudToScanGraph(pcl::PointCloud<pcl::PointXYZ> &cloud, Eigen::Isometry3d sensor_pose)
  {
    // Convert cloud to octomap graph:
    ScanGraph* graph = new ScanGraph();
    octomap::Pointcloud* scan = new octomap::Pointcloud();

    //float x, y, z, roll, pitch, yaw;
    //pose6d pose(x, y, z, roll, pitch, yaw);
    Eigen::Vector3d euler = sensor_pose.rotation().eulerAngles(2, 1, 0);
    octomap::pose6d pose(sensor_pose.translation().x(), sensor_pose.translation().y(), sensor_pose.translation().z(),
                         euler[2], euler[1], euler[0]);

    for (size_t i = 0; i < cloud.points.size(); i++){
      scan->push_back(cloud.at(i).x,
                      cloud.at(i).y,
                      cloud.at(i).z);
    }
    graph->addNode(scan, pose);
    graph->connectPrevious();

    return graph;
  }
}
