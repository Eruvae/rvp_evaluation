#include "rvp_evaluation/evaluator_external_clusters.h"

namespace rvp_evaluation
{

ExternalClusterEvaluator::ExternalClusterEvaluator(std::shared_ptr<GtOctreeLoader> gtLoader) : gtLoader(gtLoader)
{
  ros::NodeHandle nh;

  if (!gtLoader)
  {
    gtLoader.reset(new GtOctreeLoader(0.005));
  }

  gt_cluster_info = getClusterInfos(gtLoader->getPclCloud(), gtLoader->getPclClusters());

  cluster_pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/capsicum_superellipsoid_detector/xyz_label_normal", 5,
                           boost::bind(&ExternalClusterEvaluator::processReceivedClusters, this, _1));
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
  std::vector<ClusterInfo<pcl::PointXYZLNormal>> clusters = getClusterInfos(cluster_pc);
  std::vector<std::pair<size_t, size_t>> pairs = computePairs(gt_cluster_info, clusters);

  ROS_INFO_STREAM("Clusters received, processing took " << (ros::Time::now() - comp_start_time));
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

    //ROS_INFO_STREAM("GT Center: " << gt_cluster.center << ", GT Volume: " << gt_cluster.volume);
    //ROS_INFO_STREAM("Center: " << det_cluster.center << ", Volume: " << det_cluster.volume);
    //ROS_INFO_STREAM("Distance: " << center_dist << ", Volume ratio: " << (det_cluster.volume / gt_cluster.volume) << ", Accuracy: " << accuracy);
  }
  boost::unique_lock lock(current_params_mtx);
  current_params.detected_clusters = detected_clusters;
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
