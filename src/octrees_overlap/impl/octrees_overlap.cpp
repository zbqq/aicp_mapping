#include "octrees_overlap.hpp"

namespace aicp {

  OctreesOverlap::OctreesOverlap(const OctreesParams& params) :
    params_(params)
  {
    tree_  = new ColorOcTree(params_.octomapResolution);

    // define colors
    yellow = new ColorOcTreeNode::Color(250,250,0);
    green = new ColorOcTreeNode::Color(0,102,0);
    blue = new ColorOcTreeNode::Color(0,0,255);
    red = new ColorOcTreeNode::Color(255,0,0);
  }

  OctreesOverlap::~OctreesOverlap() {}

  void OctreesOverlap::doConversion(pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, int cloud_idx)
  {
    cout << "=====================================================" << endl;
    cout << "Processing cloud with " << cloud_in.points.size()
         << " points" << endl;
    string current_cloud;
    if (cloud_idx == 0)
      current_cloud = "OCTOMAP_REF";
    else
      current_cloud = "OCTOMAP_IN";

    if (cloud_idx == 1)
      tree_->resetChangeDetection();

    updateOctree(cloud_in, tree_);

    colorChanges(*tree_, cloud_idx);

    /*
    // write color tree to ot
    static char octree_col_name[50];
    sprintf(octree_col_name, "octomap-colored-%d.ot", idx);
    std::string col_path  = string(octree_col_name) ;
    cout << "Saving color octree to: " << col_path << endl;;
    tree_->write(col_path);*/

    cout << "DONE!" << endl;
    cout << "====================================================" << endl;
  }

  void OctreesOverlap::updateOctree(pcl::PointCloud<pcl::PointXYZRGB> &cloud, ColorOcTree* tree){
    //  Takes about 2.5 sec to convert 400scans or 400k points into an octree

    std::cout << "Resolution: " << params_.octomapResolution << endl; // used 0.1 and blur of 0.1 was too sharp
    double maxrange = -1;
    int max_scan_no = -1;
    unsigned char compression = 0;

    // 1. Convert to octomap graph:
    ScanGraph* graph = convertPointCloudToScanGraph(cloud);

    timeval start;
    timeval stop;

    cout << "===========================\nUpdating tree...\n===========================\n";

    gettimeofday(&start, NULL);  // start timer
    unsigned int numScans = graph->size();
    unsigned int currentScan = 1;
    bool discretize = false;

    ColorOcTree* tmp_tree = new ColorOcTree(params_.octomapResolution);

    tree->enableChangeDetection(true);
    tmp_tree->enableChangeDetection(true);

    // get default sensor model values:
    ColorOcTree emptyTree(0.1);
    double clampingMin = emptyTree.getClampingThresMin();
    double clampingMax = emptyTree.getClampingThresMax();
    double probMiss = emptyTree.getProbMiss();
    double probHit = emptyTree.getProbHit();

    tree->setClampingThresMin(clampingMin);
    tree->setClampingThresMax(clampingMax);
    tree->setProbHit(probHit);
    tree->setProbMiss(probMiss);

    for (ScanGraph::iterator scan_it = graph->begin(); scan_it != graph->end(); scan_it++) {

      tree->insertPointCloud((*scan_it)->scan, (*scan_it)->pose.trans(), maxrange, false, discretize);
      tmp_tree->insertPointCloud((*scan_it)->scan, (*scan_it)->pose.trans(), maxrange, false, discretize);

      if (compression == 2){
        tree->toMaxLikelihood();
        tmp_tree->toMaxLikelihood();
        tree->prune();
        tmp_tree->prune();
      }

      if ((max_scan_no > 0) && (currentScan == (unsigned int) max_scan_no))
        break;

      currentScan++;
    }

    gettimeofday(&stop, NULL);  // stop timer

    double time_to_insert = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);

    cout << "Time to Insert: " << time_to_insert << " seconds." << endl;

    // get rid of graph in mem before doing anything fancy with tree (=> memory)
    delete graph;

    // iterate through the new nodes of the tree
    tree->expand();
    tmp_tree->expand();

    for(ColorOcTree::tree_iterator it=tmp_tree->begin_tree(),
        end=tmp_tree->end_tree(); it!= end; ++it) {
      if (it.isLeaf()) {
        point3d coord = tmp_tree->keyToCoord(it.getKey());
        if (tmp_tree->isNodeOccupied(*it)) {

          ColorOcTreeNode* n = tree->setNodeValue(coord, true);
          n = tree->updateNode(coord, true);

          n->setColor(*blue); // set color to blue
        }
      }
    }
    tree->updateInnerOccupancy();

    /*
    //DEBUGGING:
    std::string old_path = getDataPath();
    static char octree_col_name[50];
    sprintf(octree_col_name, "octomap-pre-colored.ot");
    std::string col_path  = string( old_path + "/" +  octree_col_name) ;
    cout << "Saving color octree to: " << col_path << endl;;
    tree->write(col_path);
    publishOctree(tmp_tree, "OCTOMAP");*/

    delete tmp_tree;
    tree->prune();
  }

  ScanGraph* OctreesOverlap::convertPointCloudToScanGraph(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
  {
    // Convert cloud to octomap graph:
    ScanGraph* graph = new ScanGraph();
    octomap::Pointcloud* scan = new octomap::Pointcloud();
    //float x, y, z, roll, pitch, yaw;
    //pose6d pose(x, y, z, roll, pitch, yaw);
    pose6d pose(0, 0, 0, 0, 0, 0);
    //std::cout << "Pose "<< pose << " found.\n";

    for (size_t i = 0; i < cloud.points.size(); i++){
      scan->push_back(cloud.at(i).x,
                      cloud.at(i).y,
                      cloud.at(i).z);
    }
    graph->addNode(scan, pose);
    graph->connectPrevious();

    return graph;
  }

  void OctreesOverlap::createBlurredOctree(ColorOcTree* tree_blurred)
  {
    printf("Creating blurred octree...\n");

    float res = tree_->getResolution();

    //normalize sigma from meteres to cell resolution
    double blurVar = params_.blurSigma * params_.blurSigma;
    //compute the size of the gaussian kernel assuming max likelihood of 255, and min of 32
    double det_var = pow(blurVar, 3); //covariance is diagonal
    double normalizer = pow(2 * M_PI, -3 / 2) * pow(det_var, -1 / 2);

    int kernel_size = 2 * params_.blurSigma / res;

    //printf("kernel size is %d\n", kernel_size);

    //create the gaussian kernel
    double kxzy0[3] = { -res * kernel_size, -res * kernel_size, -res * kernel_size };
    double kxzy1[3] = { res * kernel_size, res * kernel_size, res * kernel_size };
    double krez[3] = { res, res, res };
    occ_map::FloatVoxelMap * blurKernel = new occ_map::FloatVoxelMap(kxzy0, kxzy1, krez);
    occ_map::VoxelMap<point3d> * indexTable = new occ_map::VoxelMap<point3d>(kxzy0, kxzy1, krez);
    double xyz[3];
    int ixyz[3];
    double kernel_sum = 0;
    for (ixyz[2] = 0; ixyz[2] < blurKernel->dimensions[2]; ixyz[2]++) {
      for (ixyz[1] = 0; ixyz[1] < blurKernel->dimensions[1]; ixyz[1]++) {
        for (ixyz[0] = 0; ixyz[0] < blurKernel->dimensions[0]; ixyz[0]++) {
          blurKernel->tableToWorld(ixyz, xyz);
          double val = normalizer * exp(-.5 * (bot_sq(xyz[0]) + bot_sq(xyz[1]) + bot_sq(xyz[2])) / blurVar); //diagonal cov
          kernel_sum += val;
          blurKernel->writeValue(ixyz, val);
          indexTable->writeValue(ixyz, point3d(xyz[0], xyz[1], xyz[2]));
        }
      }
    }

    //printf("kernel_sum = %f\n", kernel_sum);

    for (int i = 0; i < blurKernel->num_cells; i++) {
      blurKernel->data[i] /= kernel_sum;
    }

    double cross_section_sum = 0;
    xyz[0] = xyz[1] = xyz[2] = 0;
    blurKernel->worldToTable(xyz, ixyz);
    fprintf(stderr, "kernel = [\n");
    for (ixyz[1] = 0; ixyz[1] < blurKernel->dimensions[1]; ixyz[1]++) {
      for (ixyz[0] = 0; ixyz[0] < blurKernel->dimensions[0]; ixyz[0]++) {
        fprintf(stderr, "%f ", blurKernel->readValue(ixyz));
        cross_section_sum += blurKernel->readValue(ixyz);
      }
      fprintf(stderr, "\n");
    }
    fprintf(stderr, "];\n");

    //printf("cross_section_sum = %f\n", cross_section_sum);

    //set blurred map to occupancy probablity
    int numLeaves = tree_->getNumLeafNodes();
    int count = 0;
    for (octomap::ColorOcTree::leaf_iterator it = tree_->begin_leafs(),
        end = tree_->end_leafs(); it != end; ++it)
    {
      if (count % (numLeaves / 20) == 0) {
        //printf("%d of %d\n", count, numLeaves);
      }
      count++;

      point3d center = it.getCoordinate();
      tree_->search(it.getKey());
      if (!tree_->isNodeOccupied(tree_->search(it.getKey()))) {
        //fprintf(stderr, "skipping unoccupied node at %f, %f, %f\n", it.getCoordinate().x(),
        //        it.getCoordinate().y(), it.getCoordinate().z());
        continue;
      }
      for (int i = 0; i < blurKernel->num_cells; i++) {
        OcTreeKey key;
        if (!tree_blurred->coordToKeyChecked(center + indexTable->data[i], key)) {
          fprintf(stderr, "Error: couldn't generate key in blurred map!\n");
        }
        point3d coord = tree_->keyToCoord(it.getKey());
        ColorOcTreeNode* new_node = tree_blurred->setNodeValue(coord, true);
        new_node = tree_blurred->updateNode(key, blurKernel->data[i], true);

        ColorOcTreeNode* old_node = tree_->search(it.getKey());
        new_node->setColor(old_node->getColor());
      }
    }

    tree_blurred->updateInnerOccupancy();

    //convert from probabilities to log odds, capping at the likelihood of a wall
    int numBlurLeaves = tree_blurred->getNumLeafNodes();
    //printf("Converting to Log Odds.\n");
    count = 0;
    for (octomap::ColorOcTree::leaf_iterator it = tree_blurred->begin_leafs(),
        end = tree_blurred->end_leafs(); it != end; ++it)
    {
      octomap::ColorOcTreeNode &node = *it;
      node.setValue(-log(fmin(cross_section_sum, node.getValue())));
      /*if (node.getValue() <= 1.0 && node.getColor() == *blue)
        node.setColor(*red);
      printf("Value = %f\n", node.getValue());*/
    }
    double minNegLogLike = -log(cross_section_sum);
    printf("minNegLogLike = %f\n", minNegLogLike);
  }

  void OctreesOverlap::colorChanges(ColorOcTree& tree, int idx){
    unsigned int changedOccupied = 0;
    unsigned int changedFree = 0;
    unsigned int missingChanged = 0;

    tree.expand();

    // iterate through the changed nodes
    KeyBoolMap::const_iterator it;
    bool tmp;
    for (it=tree.changedKeysBegin(); it!=tree.changedKeysEnd(); it++) {
      ColorOcTreeNode* node = tree.search(it->first);
      if (node != NULL)
      {
        if (tree.isNodeOccupied(node)) {
          // new occupied space
          if (idx == 0)
            node->setColor(*yellow); // set color to yellow
          else{
            node->setColor(*green); // set color to green
          }
          changedOccupied += 1;
        }
        else {
          // rays from center to detected surface (new free space, unknown before).
          changedFree += 1;
        }
      }
      else
      {
        missingChanged +=1;
      }
    }

    cout << "----------non leaves included----------" << endl;
    cout << "Colored nodes (new occ space): " << changedOccupied << endl;
    cout << "Empty nodes (new free space): " << changedFree << endl;
    cout << "---------------------------------------" << endl;

    tree.prune();
  }

//  void OctreesOverlap::publishOctree(ColorOcTree* tree, string octree_channel){

//    /*
//    double minX, minY, minZ, maxX, maxY, maxZ;
//    tree->getMetricMin(minX, minY, minZ);
//    tree->getMetricMax(maxX, maxY, maxZ);
//    printf("\nmap bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]  res: %f\n", minX, minY, minZ, maxX, maxY, maxZ,
//        tree->getResolution());*/

//    octomap_raw_t msg;
//    msg.utime = bot_timestamp_now();

//    for (int i = 0; i < 4; ++i) {
//      for (int j = 0; j < 4; ++j) {
//        msg.transform[i][j] = 0;
//      }
//      msg.transform[i][i] = 1;
//    }

//    std::stringstream datastream;
//    tree->write(datastream);
//    std::string datastring = datastream.str();
//    msg.data = (uint8_t *) datastring.c_str();
//    msg.length = datastring.size();

//    octomap_raw_t_publish(lcm_->getUnderlyingLCM(), octree_channel.c_str(), &msg);
//  }

//  void OctreesOverlap::publishOctree(OcTree* tree, string octree_channel){

//    /*
//    double minX, minY, minZ, maxX, maxY, maxZ;
//    tree->getMetricMin(minX, minY, minZ);
//    tree->getMetricMax(maxX, maxY, maxZ);
//    printf("\nmap bounds: [%.2f, %.2f, %.2f] - [%.2f, %.2f, %.2f]  res: %f\n", minX, minY, minZ, maxX, maxY, maxZ,
//        tree->getResolution());*/

//    octomap_raw_t msg;
//    msg.utime = bot_timestamp_now();

//    for (int i = 0; i < 4; ++i) {
//      for (int j = 0; j < 4; ++j) {
//        msg.transform[i][j] = 0;
//      }
//      msg.transform[i][i] = 1;
//    }

//    std::stringstream datastream;
//    tree->write(datastream);
//    std::string datastring = datastream.str();
//    msg.data = (uint8_t *) datastring.c_str();
//    msg.length = datastring.size();

//    octomap_raw_t_publish(lcm_->getUnderlyingLCM(), octree_channel.c_str(), &msg);
//  }

  void OctreesOverlap::printChangesByColor(ColorOcTree& tree){
    unsigned int fromFreeToOccupied = 0;
    unsigned int fromOccupiedToFree = 0;
    unsigned int alreadyOccupied = 0;

    tree.expand();

    // iterate through the entire tree
    for(ColorOcTree::tree_iterator it=tree.begin_tree(),
        end=tree.end_tree(); it!= end; ++it) {
      if (it.isLeaf()) {
        //NOTE: leaves have size = resolution.
        ColorOcTreeNode* node = tree.search(it.getKey());
        ColorOcTreeNode::Color c = node->getColor();
        if (tree.isNodeOccupied(node)) {
          if (c == *blue)
            alreadyOccupied += 1;
          else if (c == *green)
            fromFreeToOccupied += 1;
          else if (c == *yellow)
            fromOccupiedToFree += 1;
        }
      }
    }

    cout << "----------non leaves excluded----------" << endl;
    cout << "fromFreeToOccupied: " << fromFreeToOccupied << endl;
    cout << "fromOccupiedToFree: " << fromOccupiedToFree << endl;
    cout << "alreadyOccupied: " << alreadyOccupied << endl;
    cout << "---------------------------------------" << endl;

    tree.prune();
  }

  void OctreesOverlap::printChangesAndActual(ColorOcTree& tree){
    unsigned int changedOccupied = 0;
    unsigned int changedFree = 0;
    unsigned int actualOccupied = 0;
    unsigned int actualFree = 0;
    unsigned int missingChanged = 0;

    tree.expand();

    // iterate through the changed nodes
    KeyBoolMap::const_iterator it;
    for (it=tree.changedKeysBegin(); it!=tree.changedKeysEnd(); it++) {
      ColorOcTreeNode* node = tree.search(it->first);
      if (node != NULL) {
        if (tree.isNodeOccupied(node)) {
          changedOccupied += 1;
        }
        else {
          changedFree += 1;
        }
      } else {
        missingChanged +=1;
      }
    }

    unsigned int notLeaf = 0;

    // iterate through the entire tree
    for(ColorOcTree::tree_iterator it=tree.begin_tree(),
        end=tree.end_tree(); it!= end; ++it) {
      if (it.isLeaf()) {
        if (tree.isNodeOccupied(*it)) {
          actualOccupied += 1;
        }
        else {
          actualFree += 1;
        }
      }
      else
        notLeaf += 1;
    }

    cout << "Expanded tree size: " << tree.size() <<" nodes." << endl;

    cout << "Change detection: " << changedOccupied << " occ; " << changedFree << " free; "<< missingChanged << " missing;" << endl;
    cout << "Number of changes: " << changedOccupied+changedFree << endl;
    cout << "Actual: " << actualOccupied << " occ; " << actualFree << " free; " << notLeaf << " not leaf; " << endl;
    cout << "Number of actual: " << actualOccupied+actualFree+notLeaf << endl;

    tree.prune();
  }
}
