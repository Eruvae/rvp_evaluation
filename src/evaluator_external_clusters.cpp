#include "rvp_evaluation/evaluator_external_clusters.h"

namespace rvp_evaluation
{

ExternalClusterEvaluator::ExternalClusterEvaluator(std::shared_ptr<GtOctreeLoader> gtLoader, bool use_superellipsoids) : gtLoader(gtLoader)
{
  ros::NodeHandle nh;

  if (!gtLoader)
  {
    gtLoader.reset(new GtOctreeLoader(0.005));
  }

  if (!use_superellipsoids)
  {
    cluster_pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/capsicum_superellipsoid_detector/xyz_label_normal", 2,
                             boost::bind(&ExternalClusterEvaluator::processReceivedClusters, this, boost::placeholders::_1));
    gt_cluster_info = getClusterInfos(gtLoader->getPclCloud(), gtLoader->getPclClusters());
  }
  else
  {
    superellipsoids_sub = nh.subscribe<superellipsoid_msgs::SuperellipsoidArray>("/capsicum_superellipsoid_detector/superellipsoids", 2,
                          boost::bind(&ExternalClusterEvaluator::processReceivedSuperellipsoids, this, boost::placeholders::_1));
    gt_superellipsoids_surface_pub = nh.advertise<sensor_msgs::PointCloud2>("gt_superellipsoids_surface", 2, true);

    const std::vector<superellipsoid::Superellipsoid<pcl::PointXYZ>> &gt_superellipsoids = *(gtLoader->getSuperellipsoids());

    for (const superellipsoid::Superellipsoid<pcl::PointXYZ> &se : gt_superellipsoids)
    {
      ClusterInfo ci;
      ci.center = se.getOptimizedCenter();
      ci.volume = se.computeVolume();

      superellipsoid_msgs::Superellipsoid se_msg = se.generateRosMessage();
      ci.volume_bbx = se_msg.a * se_msg.b * se_msg.c;
      gt_cluster_info.push_back(ci);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr gt_superellipsoid_surface(new pcl::PointCloud<pcl::PointXYZ>);
    for (const superellipsoid::Superellipsoid<pcl::PointXYZ> &se : gt_superellipsoids)
    {
      *(gt_superellipsoid_surface) += *(se.sampleSurface());
    }
    sensor_msgs::PointCloud2::Ptr gt_superellipsoid_surface_ros(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*gt_superellipsoid_surface, *gt_superellipsoid_surface_ros);
    gt_superellipsoid_surface_ros->header.frame_id = "world";
    gt_superellipsoid_surface_ros->header.stamp = ros::Time::now();
    gt_superellipsoids_surface_pub.publish(gt_superellipsoid_surface_ros);
  }
}

void ExternalClusterEvaluator::computeCurrentParams(const std::vector<ClusterInfo> &clusters)
{
  std::vector<std::pair<size_t, size_t>> pairs = computePairs(gt_cluster_info, clusters);

  std::vector<double> distances(gt_cluster_info.size());
  std::vector<double> volume_accuracies(gt_cluster_info.size());
  std::vector<double> volume_accuracies_bbx(gt_cluster_info.size());
  std::vector<double> volume_ratios(gt_cluster_info.size());
  std::vector<double> volume_ratios_bbx(gt_cluster_info.size());

  size_t detected_clusters = 0;
  double centers_sum = 0;
  double volume_acc_sum = 0;
  double volume_acc_bbx_sum = 0;
  double volume_ratio_sum = 0;
  double volume_ratio_bbx_sum = 0;
  for (const auto &pair : pairs)
  {

    const auto &gt_cluster = gt_cluster_info[pair.first];
    const auto &det_cluster = clusters[pair.second];

    double center_dist = static_cast<double>(pcl::L2_Norm(gt_cluster.center.getArray3fMap(), det_cluster.center.getArray3fMap(), 3));
    double accuracy = 1 - std::abs(det_cluster.volume - gt_cluster.volume) / gt_cluster.volume;
    double accuracy_bbx = 1 - std::abs(det_cluster.volume_bbx - gt_cluster.volume_bbx) / gt_cluster.volume_bbx;
    double ratio = det_cluster.volume / gt_cluster.volume;
    double ratio_bbx = det_cluster.volume_bbx / gt_cluster.volume_bbx;
    detected_clusters++;
    centers_sum += center_dist;
    volume_acc_sum += accuracy;
    volume_acc_bbx_sum += accuracy_bbx;
    volume_ratio_sum += ratio;
    volume_ratio_bbx_sum += ratio_bbx;

    distances[pair.first] = center_dist;
    volume_accuracies[pair.first] = accuracy;
    volume_accuracies_bbx[pair.first] = accuracy_bbx;
    volume_ratios[pair.first] = ratio;
    volume_ratios_bbx[pair.first] = ratio_bbx;

    //ROS_INFO_STREAM("GT Center: " << gt_cluster.center << ", GT Volume: " << gt_cluster.volume);
    //ROS_INFO_STREAM("Center: " << det_cluster.center << ", Volume: " << det_cluster.volume);
    //ROS_INFO_STREAM("Distance: " << center_dist << ", Volume ratio: " << (det_cluster.volume / gt_cluster.volume) << ", Accuracy: " << accuracy);
  }

  boost::unique_lock lock(current_params_mtx);
  current_params.detected_clusters = detected_clusters;
  current_params.distances = distances;
  current_params.volume_accuracies = volume_accuracies;
  current_params.volume_accuracies_bbx = volume_accuracies;
  current_params.volume_ratios = volume_ratios;
  current_params.volume_ratios_bbx = volume_ratios_bbx;

  if (detected_clusters > 0)
  {
    current_params.center_distance = centers_sum / detected_clusters;
    current_params.volume_accuracy = volume_acc_sum / detected_clusters;
    current_params.volume_accuracy_bbx = volume_acc_bbx_sum / detected_clusters;
    current_params.volume_ratio = volume_ratio_sum / detected_clusters;
    current_params.volume_ratio_bbx = volume_ratio_bbx_sum / detected_clusters;
  }
  else
  {
    current_params.center_distance = 0;
    current_params.volume_accuracy = 0;
    current_params.volume_accuracy_bbx = 0;
    current_params.volume_ratio = 0;
    current_params.volume_ratio_bbx = 0;
  }
}

void ExternalClusterEvaluator::processReceivedClusters(const sensor_msgs::PointCloud2::ConstPtr &cluster_pc_msg)
{
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cluster_pc(new pcl::PointCloud<pcl::PointXYZLNormal>);
  pcl::fromROSMsg(*cluster_pc_msg, *cluster_pc);

  if (cluster_pc->size() < 3) // Minimum 3 points necessary for meaningful computations
  {
    ROS_ERROR_STREAM("Received PC to small (" << cluster_pc->size() << " points)");
    return;
  }

  ros::Time comp_start_time = ros::Time::now();
  std::vector<ClusterInfo> clusters = getClusterInfos(cluster_pc);
  ROS_INFO_STREAM("Clusters received, processing took " << (ros::Time::now() - comp_start_time));

  computeCurrentParams(clusters);
}

void ExternalClusterEvaluator::processReceivedSuperellipsoids(const superellipsoid_msgs::SuperellipsoidArray::ConstPtr &se_arr_msg)
{
  std::vector<ClusterInfo> clusters(se_arr_msg->superellipsoids.size());
  for (size_t i=0; i < clusters.size(); i++)
  {
    const superellipsoid_msgs::Superellipsoid &se_msg = se_arr_msg->superellipsoids[i];
    //superellipsoid::Superellipsoid<pcl::PointXYZ> se(se_msg);
    //clusters[i].center = se.getOptimizedCenter();
    //clusters[i].volume = se.computeVolume();
    clusters[i].center = pcl::PointXYZ(static_cast<float>(se_msg.tx), static_cast<float>(se_msg.ty), static_cast<float>(se_msg.tz));
    clusters[i].volume = se_msg.volume;
    clusters[i].volume_bbx = se_msg.a * se_msg.b * se_msg.c;
  }

  computeCurrentParams(clusters);
}

ECEvalParams ExternalClusterEvaluator::getCurrentParams()
{
  boost::unique_lock lock(current_params_mtx);
  return current_params;
}

std::ostream& ExternalClusterEvaluator::writeHeader(std::ostream &os)
{
  os << "Detected ROI cluster,Dist.,Vol. acc.,Vol. acc. bbx,Ratio,Ratio bbx";
  return os;
}

std::ostream& ExternalClusterEvaluator::writeParams(std::ostream &os, const ECEvalParams &res)
{
  /*size_t detected_clusters = 0;
  double center_distance = 0;
  double volume_accuracy = 0;*/
  os << res.detected_clusters << "," << res.center_distance << "," << res.volume_accuracy << "," << res.volume_accuracy_bbx << ","
     << res.volume_ratio << "," << res.volume_ratio_bbx;
  return os;
}

} // namespace rvp_evaluation
