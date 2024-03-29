#include "rvp_evaluation/evaluator.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace rvp_evaluation
{

Evaluator::Evaluator(std::shared_ptr<OctreeProviderInterface> planner, ros::NodeHandle &nh, ros::NodeHandle &nhp, bool gt_comparison, bool use_pcl_visualizer, std::shared_ptr<GtOctreeLoader> gtLoader)
  : planner(planner), gtLoader(gtLoader), gt_comparison(gt_comparison), use_pcl_visualizer(use_pcl_visualizer),
    viewer(nullptr), vp1(0), vp2(1), viewer_initialized(false), server(nhp),
    visualizeThread(&Evaluator::visualizeLoop, this)
{
  gt_pub = nhp.advertise<visualization_msgs::Marker>("roi_gt", 10, true);
  gt_fruit_pub = nhp.advertise<octomap_msgs::Octomap>("gt_fruits", 10, true);
  gt_fruits_inflated_pub = nhp.advertise<octomap_msgs::Octomap>("gt_fruits_inflated", 10, true);
  if (gt_comparison)
  {
    if (!nh.param<std::string>("/world_name", world_name, ""))
    {
      ROS_WARN("World name not specified, cannot load ground truth; Evaluator state invalid");
      return;
    }
    else if (!readGroundtruth())
    {
      ROS_WARN("Groundtruth could not be read; Evaluator state invalid");
      return;
    }

    // Register publishers and subscribers

    publishCubeVisualization(gt_pub, *roi_locations, *roi_sizes, COLOR_GREEN, "gt_rois");
  }

  if (use_pcl_visualizer)
  {
    while(!viewer_initialized.load())
    {
      ROS_INFO("Waiting for viewer initialization...");
      ros::Duration(0.1).sleep();
    }

    updateVisualizerGroundtruth();
  }

  server.setCallback(boost::bind(&Evaluator::reconfigureCallback, this, boost::placeholders::_1, boost::placeholders::_2));
}

bool Evaluator::readGroundtruth()
{
  if (!gt_comparison)
    return false;

  if (gtLoader == nullptr)
    gtLoader.reset(new GtOctreeLoader(planner->getTreeResolution()));

  octomap_msgs::Octomap fruit_ot_msg;
  fruit_ot_msg.header.frame_id = "world";
  fruit_ot_msg.header.stamp = ros::Time::now();
  std::shared_ptr<const octomap_vpp::CountingOcTree> gt_fruits = gtLoader->getIndexedFruitTree();
  bool msg_generated = octomap_msgs::fullMapToMsg(*gt_fruits, fruit_ot_msg);
  if (msg_generated)
  {
    gt_fruit_pub.publish(fruit_ot_msg);
  }
  std::shared_ptr<octomap_vpp::NearestRegionOcTree> gt_fruits_inflated = octomap_vpp::NearestRegionOcTree::createFromCountringOctree(*gt_fruits, 0.2);
  msg_generated = octomap_msgs::fullMapToMsg(*gt_fruits_inflated, fruit_ot_msg);
  if (msg_generated)
  {
    gt_fruits_inflated_pub.publish(fruit_ot_msg);
  }

  gt_pcl = gtLoader->getPclCloud();
  gt_clusters = gtLoader->getPclClusters();
  computeHullsAndVolumes(gt_pcl, *gt_clusters, gt_hulls_clouds, gt_hulls_polygons, gt_params.fruit_volumes, gt_centroids);

  gt_params.num_fruits = gtLoader->getNumFruits();
  gt_params.fruit_cell_counts = gtLoader->getFruitCellsCounts();

  // Legacy groundtruth for processDetectedRoisOld

  roi_locations = gtLoader->getFruitLocations();
  roi_sizes = gtLoader->getFruitSizes();

  // Compute GT-keys from bounding boxes

  gtRoiKeys.clear();
  for (size_t i = 0; i < roi_locations->size(); i++)
  {
    octomap::OcTreeKey minKey = planner->getPlanningTree()->coordToKey((*roi_locations)[i] - (*roi_sizes)[i] * 0.5);
    octomap::OcTreeKey maxKey = planner->getPlanningTree()->coordToKey((*roi_locations)[i] + (*roi_sizes)[i] * 0.5);
    octomap::OcTreeKey curKey = minKey;
    for (curKey[0] = minKey[0]; curKey[0] <= maxKey[0]; curKey[0]++)
    {
      for (curKey[1] = minKey[1]; curKey[1] <= maxKey[1]; curKey[1]++)
      {
        for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; curKey[2]++)
        {
          gtRoiKeys.insert(curKey);
        }
      }
    }
  }
  ROS_INFO_STREAM("BB ROI key count: " << gtRoiKeys.size());

  return true;
}

/*void Evaluator::computeGroundtruthPCL()
{
  if (!gt_comparison || !use_pcl_visualizer)
    return;

  gt_clusters.clear();
  gt_hulls_clouds.clear();
  gt_hulls_polygons.clear();
  gt_cluster_volumes.clear();

  clusterWithPCL(gt_pcl, gt_clusters);
  computeHullsAndVolumes(gt_pcl, gt_clusters, gt_hulls_clouds, gt_hulls_polygons, gt_cluster_volumes);
}*/

/*void Evaluator::computeDetectionsPCL()
{
  if (!use_pcl_visualizer)
    return;

  clusters.clear();
  hulls_clouds.clear();
  hulls_polygons.clear();
  cluster_volumes.clear();

  clusterWithPCL(roi_pcl, clusters);
  computeHullsAndVolumes(roi_pcl, clusters, hulls_clouds, hulls_polygons, cluster_volumes);
}*/

void Evaluator::updateVisualizerGroundtruth()
{
  if (!gt_comparison || !use_pcl_visualizer)
    return;

  gt_color_handler.reset(new pcl::visualization::PointCloudColorHandlerClusters<pcl::PointXYZ>(gt_pcl, *gt_clusters));
  viewer_mtx.lock();
  viewer->removeAllPointClouds(vp1);
  viewer->removeAllShapes(vp1);
  viewer->addPointCloud<pcl::PointXYZ> (gt_pcl, *gt_color_handler, "gt_cloud", vp1);
  if (config.show_hulls)
  {
    for (size_t i = 0; i < gt_clusters->size(); i++)
    {
      std::string name = "gt_cluster_" + std::to_string(i);
      viewer->addPolygonMesh<pcl::PointXYZ>(gt_hulls_clouds[i], gt_hulls_polygons[i], name, vp1);
    }
  }
  viewer_mtx.unlock();
}

void Evaluator::updateVisualizerDetections()
{
  if (viewer_initialized.load())
  {
    roi_color_handler.reset(new pcl::visualization::PointCloudColorHandlerClusters<pcl::PointXYZ>(roi_pcl, clusters));
    viewer_mtx.lock();
    viewer->removeAllPointClouds(vp2);
    viewer->removeAllShapes(vp2);
    viewer->addPointCloud<pcl::PointXYZ> (roi_pcl, *roi_color_handler, "roi_cloud", vp2);
    //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (roi_pcl, normal_cloud, 1, 0.02f, "roi_normals");
    if (config.show_hulls)
    {
      for (size_t i = 0; i < clusters.size(); i++)
      {
        std::string name = "cluster_" + std::to_string(i);
        viewer->addPolygonMesh<pcl::PointXYZ>(hulls_clouds[i], hulls_polygons[i], name, vp2);
      }
    }
    viewer_mtx.unlock();
  }
}

void Evaluator::reconfigureCallback(roi_viewpoint_planner::EvaluatorConfig &new_config, uint32_t level)
{
  if (new_config.min_cluster_size > new_config.max_cluster_size) // min cluster size must be smaller than max cluster size
  {
    if (level & (1 << 1)) // min_cluster_size was changed
      new_config.max_cluster_size = new_config.min_cluster_size;
    else //if (level & (1 << 2)) / max_cluster_size was changed
      new_config.min_cluster_size = new_config.max_cluster_size;
  }

  if (!new_config.curvature_test_flag && !new_config.residual_test_flag) // either curvature or residual test must be activated
  {
    if (level & (1 << 9)) // residual_test_flag was changed
      new_config.curvature_test_flag = true;
    else if (level & (1 << 8)) // curvature_test_flag was changed
      new_config.residual_test_flag = true;
  }

  config = new_config;

  if (use_pcl_visualizer)
  {
    //computeGroundtruthPCL();
    //updateVisualizerGroundtruth();
    processDetectedRois();
    updateVisualizerDetections();
  }
}

void Evaluator::visualizeLoop()
{
  if (!use_pcl_visualizer)
    return;

  viewer_mtx.lock();
  viewer = new pcl::visualization::PCLVisualizer("ROI viewer");
  if (gt_comparison)
  {
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, vp1);
    viewer->setBackgroundColor (0, 0, 0, vp1);
    viewer->addText ("Groundtruth", 10, 10, "vp1cap", vp1);
  }
  //viewer->createViewPortCamera(vp1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp2);
  viewer->setBackgroundColor (0.1, 0.1, 0.1, vp2);
  viewer->addText ("Detection", 10, 10, "vp2cap", vp2);
  //viewer->createViewPortCamera(vp2);
  viewer_initialized.store(true);
  viewer_mtx.unlock();
  for (ros::Rate rate(30); ros::ok(); rate.sleep())
  {
    viewer_mtx.lock();
    viewer->spinOnce();
    viewer_mtx.unlock();
  }
}

bool Evaluator::clusterWithPCL(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, std::vector<pcl::PointIndices> &clusters, pcl::PointCloud<pcl::Normal>::Ptr normal_cloud)
{
  if (input_cloud->empty())
  {
    ROS_WARN("Input cloud was emtpy; no clustering performed");
    return false;
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_cloud);

  if (config.clustering == roi_viewpoint_planner::Evaluator_EUCLIDEAN_CLUSTERING)
  {
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (config.cluster_tolerance);
    ec.setMinClusterSize (config.min_cluster_size);
    ec.setMaxClusterSize (config.max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_cloud);
    ec.extract (clusters);
  }
  else if (config.clustering == roi_viewpoint_planner::Evaluator_REGION_GROWING)
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(config.normal_search_radius);
    normal_cloud.reset(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normal_cloud);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
    rg.setInputCloud(input_cloud);
    rg.setSearchMethod(tree);
    rg.setInputNormals(normal_cloud);
    rg.setMinClusterSize(config.min_cluster_size);
    rg.setMaxClusterSize(config.max_cluster_size);
    rg.setSmoothnessThreshold(config.smoothness_threshold / 180.0f * M_PI);
    rg.setCurvatureThreshold(config.curvature_threshold);
    rg.setResidualThreshold(config.residual_threshold);
    rg.setSmoothModeFlag(config.smooth_mode_flag);
    rg.setCurvatureTestFlag(config.curvature_test_flag);
    rg.setResidualTestFlag(config.residual_test_flag);
    rg.extract(clusters);
  }
  else
  {
    ROS_ERROR("No valid clustering method was selected");
    return false;
  }
  return true;
}

bool Evaluator::computeHullsAndVolumes(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, const std::vector<pcl::PointIndices> &clusters,
                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &hulls_clouds,
                           std::vector<std::vector<pcl::Vertices>> &hulls_polygons,
                           std::vector<double> &cluster_volumes,
                           std::vector<pcl::PointXYZ> &centroids)
{
  if (input_cloud->empty() || clusters.empty())
    return false;

  hulls_clouds.clear();
  hulls_polygons.clear();
  cluster_volumes.clear();
  centroids.clear();

  for (const pcl::PointIndices &cluster : clusters)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> hull_polygons;

    pcl::PointIndicesPtr cluster_ptr(new pcl::PointIndices(cluster));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extr_inds;
    extr_inds.setInputCloud(input_cloud);
    extr_inds.setIndices(cluster_ptr);
    extr_inds.filter(*cluster_cloud);

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cluster_cloud);
    //hull.setIndices(cluster_ptr); // doesn't work -> use extract indices
    hull.setComputeAreaVolume(true);

    hull.reconstruct(*hull_cloud, hull_polygons);
    int dim = hull.getDimension();
    double vol;
    if (dim == 2)
    {
      const double thickness = 0.01; // Assume 1 cm thickness (might be reasonable to set to octomap resolution)
      vol = hull.getTotalArea() * thickness * 1000000; // to cm3
    }
    else // if dim == 3
    {
      vol = hull.getTotalVolume() * 1000000; // to cm3
    }

    ROS_INFO_STREAM("Hull dimension: " << dim << "Cluster volume: " << vol);

    hulls_clouds.push_back(hull_cloud);
    hulls_polygons.push_back(hull_polygons);
    cluster_volumes.push_back(vol);
    pcl::PointXYZ centroid;
    pcl::computeCentroid(*input_cloud, cluster.indices, centroid);
    centroids.push_back(centroid);
  }
  return true;
}

void Evaluator::computePairsAndDistances()
{
  distances.clear();
  roiPairs.clear();

  // Build up matrix with all pairs
  distances = boost::numeric::ublas::matrix<double>(roi_locations->size(), detected_locs.size());
  std::vector<IndexPair> indices;
  indices.reserve(roi_locations->size() * detected_locs.size());
  for (size_t i = 0; i < roi_locations->size(); i++)
  {
    for (size_t j = 0; j < detected_locs.size(); j++)
    {
      distances(i, j) = ((*roi_locations)[i] - detected_locs[j]).norm();
      indices.push_back(IndexPair(i, j));
    }
  }

  auto indicesComp = [this](const IndexPair &a, const IndexPair &b)
  {
    return distances(a.gt_ind, a.det_ind) > distances(b.gt_ind, b.det_ind);
  };

  boost::dynamic_bitset<> usedGtPoints(roi_locations->size());
  boost::dynamic_bitset<> usedDetPoints(detected_locs.size());

  roiPairs.reserve(std::min(roi_locations->size(), detected_locs.size()));

  double MAX_DISTANCE = 0.2; // Maximal distance to be considered as same ROI

  for (std::make_heap(indices.begin(), indices.end(), indicesComp); !usedGtPoints.all() && !usedDetPoints.all(); std::pop_heap(indices.begin(), indices.end(), indicesComp), indices.pop_back())
  {
    const IndexPair &pair = indices.front();

    if (distances(pair.gt_ind, pair.det_ind) > MAX_DISTANCE)
      break;

    if (usedGtPoints.test(pair.gt_ind) || usedDetPoints.test(pair.det_ind))
      continue;

    roiPairs.push_back(pair);
    usedGtPoints.set(pair.gt_ind);
    usedDetPoints.set(pair.det_ind);
  }
}

EvaluationParametersOld Evaluator::processDetectedRoisOld()
{
  EvaluationParametersOld results;
  std::shared_ptr<octomap_vpp::RoiOcTree> roiTree = planner->getPlanningTree();
  if (!roiTree)
    return results;

  planner->getTreeMutex().lock();
  roiTree->computeRoiKeys();
  octomap::KeySet roi_keys = roiTree->getRoiKeys();
  std::tie(detected_locs, detected_sizes) = roiTree->getClusterCentersWithVolume();

  // Mesh computations
  /*std::vector<octomap::point3d> vertices;
  std::vector<octomap_vpp::Triangle> faces;
  auto isRoi = [](const octomap_vpp::RoiOcTree &tree, const octomap_vpp::RoiOcTreeNode *node) { return tree.isNodeROI(node); };
  auto isOcc = [](const octomap_vpp::RoiOcTree &tree, const octomap_vpp::RoiOcTreeNode *node) { return node->getLogOdds() > 0; };
  octomap_vpp::polygonizeSubset<octomap_vpp::RoiOcTree>(*roiTree, roi_keys, vertices, faces, isRoi);
  // PCL Tests
  auto faceClusters = octomap_vpp::computeFaceClusters(faces);
  auto vertexClusters = octomap_vpp::computeClusterVertices(vertices, faceClusters);
  for (const octomap::point3d_collection &coll : vertexClusters)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud = octomap_vpp::octomapPointCollectionToPcl<pcl::PointXYZ>(coll);
    ROS_INFO_STREAM("Cluster cloud size: " << cluster_cloud->size());
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = octomap_vpp::octomapToPcl<octomap_vpp::RoiOcTree, pcl::PointXYZ>(*roiTree, isOcc);
  ROS_INFO_STREAM(pcl_cloud->size());*/

  //auto isRoi = [](const octomap_vpp::RoiOcTree &tree, const octomap_vpp::RoiOcTreeNode *node) { return tree.isNodeROI(node); };
  //roi_pcl = octomap_vpp::octomapToPcl<octomap_vpp::RoiOcTree, pcl::PointXYZ>(*roiTree, isRoi);

  planner->getTreeMutex().unlock();

  //publishCubeVisualization(gt_pub, detected_locs, detected_sizes, COLOR_RED, "detected_rois");

  /*if (use_pcl_visualizer)
  {
    computeDetectionsPCL();
    updateVisualizerDetections();
  }*/

  // ROS_INFO_STREAM("GT-Rois: " << roi_locations->size() << ", detected Rois: " << detected_locs.size());

  double average_dist = 0;
  double average_vol_accuracy = 0;

  size_t detected_clusters_ma0 = 0;
  double average_dist_ma0 = 0;
  double average_vol_accuracy_ma0 = 0;

  size_t detected_clusters_ma50 = 0;
  double average_dist_ma50 = 0;
  double average_vol_accuracy_ma50 = 0;

  if (gt_comparison)
  {
    computePairsAndDistances(); // computes roiPairs and distances

    ROS_INFO_STREAM("Closest point pairs:");
    for (const IndexPair &pair : roiPairs)
    {
      ROS_INFO_STREAM((*roi_locations)[pair.gt_ind] << " and " << detected_locs[pair.det_ind] << "; distance: " << distances(pair.gt_ind, pair.det_ind));
      double gs = (*roi_sizes)[pair.gt_ind].x() * (*roi_sizes)[pair.gt_ind].y() * (*roi_sizes)[pair.gt_ind].z();
      double ds = detected_sizes[pair.det_ind].x() * detected_sizes[pair.det_ind].y() * detected_sizes[pair.det_ind].z();

      double accuracy = 1 - std::abs(ds - gs) / gs;

      ROS_INFO_STREAM("Sizes: " << (*roi_sizes)[pair.gt_ind] << " and " << detected_sizes[pair.det_ind] << "; factor: " << (ds / gs));

      average_dist += distances(pair.gt_ind, pair.det_ind);
      average_vol_accuracy += accuracy;

      if (accuracy > 0)
      {
        detected_clusters_ma0++;
        average_dist_ma0 += distances(pair.gt_ind, pair.det_ind);
        average_vol_accuracy_ma0 += accuracy;
      }
      if (accuracy > 0.5)
      {
        detected_clusters_ma50++;
        average_dist_ma50 += distances(pair.gt_ind, pair.det_ind);
        average_vol_accuracy_ma50 += accuracy;
      }

    }

    if (roiPairs.size() > 0)
    {
      average_dist /= roiPairs.size();
      average_vol_accuracy /= roiPairs.size();
    }
    if (detected_clusters_ma0 > 0)
    {
      average_dist_ma0 /= detected_clusters_ma0;
      average_vol_accuracy_ma0 /= detected_clusters_ma0;
    }
    if (detected_clusters_ma50 > 0)
    {
      average_dist_ma50 /= detected_clusters_ma50;
      average_vol_accuracy_ma50 /= detected_clusters_ma50;
    }
  }

  // Compute volume overlap

  octomap::KeySet detectedRoiBBkeys;
  octomap::KeySet correctRoiBBkeys;
  octomap::KeySet falseRoiBBkeys;
  for (size_t i = 0; i < detected_locs.size(); i++)
  {
    octomap::OcTreeKey minKey = roiTree->coordToKey(detected_locs[i] - detected_sizes[i] * 0.5);
    octomap::OcTreeKey maxKey = roiTree->coordToKey(detected_locs[i] + detected_sizes[i] * 0.5);
    octomap::OcTreeKey curKey = minKey;
    for (curKey[0] = minKey[0]; curKey[0] <= maxKey[0]; curKey[0]++)
    {
      for (curKey[1] = minKey[1]; curKey[1] <= maxKey[1]; curKey[1]++)
      {
        for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; curKey[2]++)
        {
          detectedRoiBBkeys.insert(curKey);
          if (gt_comparison)
          {
            if (gtRoiKeys.find(curKey) != gtRoiKeys.end())
            {
              correctRoiBBkeys.insert(curKey);
            }
            else
            {
              falseRoiBBkeys.insert(curKey);
            }
          }
        }
      }
    }
  }
  //ROS_INFO_STREAM("Detected key count: " << detectedRoiBBkeys.size() << ", correct: " << correctRoiBBkeys.size() << ", false: " << falseRoiBBkeys.size());


  double coveredRoiVolumeRatio = 0;
  double falseRoiVolumeRatio = 0;
  if (gt_comparison)
  {
    coveredRoiVolumeRatio = (double)correctRoiBBkeys.size() / (double)gtRoiKeys.size();
    falseRoiVolumeRatio = detectedRoiBBkeys.size() ? (double)falseRoiBBkeys.size() / (double)detectedRoiBBkeys.size() : 0;
    //ROS_INFO_STREAM("Det vol ratio: " << coveredRoiVolumeRatio << ", False vol ratio: " << falseRoiVolumeRatio);
  }

  // Computations directly with ROI cells
  octomap::KeySet true_roi_keys, false_roi_keys;
  if (gt_comparison)
  {
    for (const octomap::OcTreeKey &key : roi_keys)
    {
      if (gtRoiKeys.find(key) != gtRoiKeys.end())
      {
        true_roi_keys.insert(key);
      }
      else
      {
        false_roi_keys.insert(key);
      }
    }
  }

  //resultsFile << "Time (s), Detected ROIs, ROI percentage, Average distance, Average volume accuracy, Covered ROI volume, False ROI volume, ROI key count, True ROI keys, False ROI keys" << std::endl;

  results.total_roi_clusters = detected_locs.size();
  results.roi_key_count = roi_keys.size();

  if (gt_comparison)
  {
    results.detected_roi_clusters = roiPairs.size();
    results.roi_percentage = (double)roiPairs.size() / (double)roi_locations->size();
    results.average_distance = average_dist;
    results.average_accuracy = average_vol_accuracy;
    results.covered_roi_volume = coveredRoiVolumeRatio;
    results.false_roi_volume = falseRoiVolumeRatio;
    results.true_roi_key_count = true_roi_keys.size();
    results.false_roi_key_count = false_roi_keys.size();

    results.detected_roi_clusters_ma0 = detected_clusters_ma0;
    results.average_accuracy_ma0 = average_vol_accuracy_ma0;
    results.average_distance_ma0 = average_dist_ma0;

    results.detected_roi_clusters_ma50 = detected_clusters_ma50;
    results.average_accuracy_ma50 = average_vol_accuracy_ma50;
    results.average_distance_ma50 = average_dist_ma50;
  }

  return results;
}

EvaluationParameters Evaluator::processDetectedRois(bool save_pointcloud, size_t trial_num, size_t step)
{
  EvaluationParameters results;
  std::shared_ptr<octomap_vpp::RoiOcTree> roiTree = planner->getPlanningTree();
  if (!roiTree)
    return results;

  planner->getTreeMutex().lock();
  roiTree->computeRoiKeys();
  octomap::KeySet roi_keys = roiTree->getRoiKeys();
  planner->getTreeMutex().unlock();

  /*size_t detected_roi_clusters;
  double average_distance;
  double average_accuracy;
  double covered_roi_volume;
  size_t roi_key_count;
  size_t true_roi_key_count;
  size_t false_roi_key_count;
  std::vector<double> fruit_cell_counts;
  std::vector<double> fruit_cell_percentages;
  std::vector<double> volume_accuracies;
  std::vector<double> distances;
  std::vector<double> volumes;*/

  results.roi_key_count = roi_keys.size();

  // Computations directly with ROI cells
  if (gt_comparison)
  {
    results.fruit_cell_counts.resize(gt_params.num_fruits);
    results.fruit_cell_percentages.resize(gt_params.num_fruits);
    results.volume_accuracies.resize(gt_params.num_fruits);
    results.distances.resize(gt_params.num_fruits);
    roi_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>());
    clusters.clear();
    clusters.resize(gt_params.num_fruits);
    for (const octomap::OcTreeKey &key : roi_keys)
    {
      // new per-fruit comparison
      unsigned int ind = gtLoader->getFruitIndex(key);
      if (ind != 0)
      {
        octomap::point3d coord = planner->getPlanningTree()->keyToCoord(key);
        clusters[ind - 1].indices.push_back(roi_pcl->size());
        roi_pcl->push_back(octomap_vpp::octomapPointToPcl<pcl::PointXYZ>(coord));
        results.fruit_cell_counts[ind - 1]++;
        results.true_roi_key_count++;
      }
      else
      {
        results.false_roi_key_count++;
      }
    }

    if (save_pointcloud)
    {
      std::stringstream filename;
      filename << "pointcloud_" << trial_num << "_" << step << ".pcd";
      saveClustersAsColoredCloud(filename.str(), roi_pcl, clusters);
    }
    {
      std_srvs::Empty srv;
      auto generate_mesh_client = nh_.serviceClient<std_srvs::Empty>("/voxblox_node/generate_mesh");
      if (generate_mesh_client.call(srv))
      {
        ROS_WARN("Voxblox mesh generated successfully");
        char oldname[] = "/home/rohit/workspace/ros1/rvp-ros-workspaces/ros_rvp/devel_release/voxblox_mesh.ply";
        std::stringstream newname;
        newname<< "voxblox_mesh_" << trial_num << "_" << step << ".ply";
        /*	Deletes the file if exists */
        if (rename(oldname, newname.str().c_str()) != 0)
          ROS_ERROR("Error renaming file");
        else
          ROS_WARN("File renamed successfully");
      }
      else
      {
        ROS_ERROR("Failed to call generate voxbox mesh service");
      }
    }

    std::vector<pcl::PointXYZ> centroids;
    computeHullsAndVolumes(roi_pcl, clusters, hulls_clouds, hulls_polygons, results.volumes, centroids);

    for (size_t i = 0; i < gt_params.num_fruits; i++)
    {
      if (results.fruit_cell_counts[i] > 0)
      {
        results.detected_roi_clusters++;
        results.fruit_cell_percentages[i] = (double)results.fruit_cell_counts[i] / (double)gtLoader->getNumFruitCells(i);
        results.distances[i] = pcl::squaredEuclideanDistance(gt_centroids[i], centroids[i]);
        results.volume_accuracies[i] = results.volumes[i] / gt_params.fruit_volumes[i];
        results.average_accuracy += results.volume_accuracies[i];
        results.covered_roi_volume += results.volume_accuracies[i];
        results.average_distance += results.distances[i];
      }

    }
    results.average_distance /= results.detected_roi_clusters;
    results.average_accuracy /= results.detected_roi_clusters;
    results.covered_roi_volume /= gt_params.num_fruits;
  }

  results.roi_key_count = roi_keys.size();

  return results;
}

void Evaluator::saveClustersAsColoredCloud(const std::string &filename, pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, const std::vector<pcl::PointIndices> &clusters)
{
  if (!input_cloud || input_cloud->empty())
    return;

  pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
  colored_cloud.reserve(input_cloud->size());
  for (size_t cid=0; cid < clusters.size(); cid++)
  {
    pcl::RGB color = pcl::GlasbeyLUT::at(cid % pcl::GlasbeyLUT::size());
    for (int ind : clusters[cid].indices)
    {
      pcl::PointXYZRGB p(color.r, color.b, color.g);
      p.x = input_cloud->at(ind).x;
      p.y = input_cloud->at(ind).y;
      p.z = input_cloud->at(ind).z;
      colored_cloud.push_back(p);
    }
  }

  if (colored_cloud.empty())
    return;

  pcl::io::savePCDFile(filename, colored_cloud, true);
}

ostream& Evaluator::writeHeaderOld(ostream &os)
{
  os << "Detected ROI cluster,Total ROI cluster,ROI percentage,Average distance,Average volume accuracy,Covered ROI volume,False ROI volume,ROI key count,True ROI keys,False ROI keys,Det. cluster ma0,Dist. ma0,Vol. acc. ma0,Det. cluster ma50,Dist. ma50,Vol. acc. ma50";
  return os;
}

ostream& Evaluator::writeParamsOld(ostream &os, const EvaluationParametersOld &res)
{
  /*size_t detected_roi_clusters;
  size_t total_roi_clusters;
  double roi_percentage;
  double average_distance;
  double average_accuracy;
  double covered_roi_volume;
  double false_roi_volume;
  size_t roi_key_count;
  size_t true_roi_key_count;
  size_t false_roi_key_count;*/
  os << res.detected_roi_clusters << "," << res.total_roi_clusters << "," << res.roi_percentage << "," << res.average_distance
     << "," << res.average_accuracy << "," << res.covered_roi_volume << "," << res.false_roi_volume
     << "," << res.roi_key_count << "," << res.true_roi_key_count << "," << res.false_roi_key_count
     << "," << res.detected_roi_clusters_ma0 << "," << res.average_distance_ma0 << "," << res.average_accuracy_ma0
     << "," << res.detected_roi_clusters_ma50 << "," << res.average_distance_ma50 << "," << res.average_accuracy_ma50;
  return os;
}

ostream& Evaluator::writeHeader(ostream &os)
{
  os << "Detected ROI cluster,Average distance,Average volume accuracy,Covered ROI volume,ROI key count,True ROI keys,False ROI keys";
  return os;
}

ostream& Evaluator::writeParams(ostream &os, const EvaluationParameters &res)
{
  /*size_t detected_roi_clusters;
  double average_distance;
  double average_accuracy;
  double covered_roi_volume;
  size_t roi_key_count;
  size_t true_roi_key_count;
  size_t false_roi_key_count;
  std::vector<double> fruit_cell_counts;
  std::vector<double> fruit_cell_percentages;
  std::vector<double> volume_accuracies;
  std::vector<double> distances;
  std::vector<double> volumes;*/
  os << res.detected_roi_clusters << "," << res.average_distance << "," << res.average_accuracy << "," << res.covered_roi_volume
     << "," << res.roi_key_count << "," << res.true_roi_key_count << "," << res.false_roi_key_count;
  return os;
}

void Evaluator::randomizePlantPositions(const octomap::point3d &min, const octomap::point3d &max, double min_dist)
{
  gtLoader->randomizePlantPositions(min, max, min_dist);
  readGroundtruth();
}

}
