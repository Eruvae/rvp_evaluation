#include <ros/ros.h>
#include "gt_octree_loader.h"
#include <sensor_msgs/PointCloud2.h>
#include <superellipsoid_msgs/SuperellipsoidArray.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/norms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/numeric/ublas/matrix.hpp>

namespace rvp_evaluation
{

template<typename PointT>
struct ClusterInfoHelper
{
  pcl::PointIndicesPtr inds = pcl::make_shared<pcl::PointIndices>();
  pcl::CentroidPoint<PointT> centroid;
  typename pcl::ConvexHull<PointT>::Ptr hull;
  typename pcl::PointCloud<PointT>::Ptr hull_cloud;
  pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
};

struct ClusterInfo
{
  pcl::PointXYZ center;
  double volume;
  double volume_bbx;
};

static std::vector<ClusterInfo> getClusterInfos(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc, const std::shared_ptr<const std::vector<pcl::PointIndices>> &inds)
{
  std::vector<ClusterInfoHelper<pcl::PointXYZ>> cluster_helpers(inds->size());
  std::vector<ClusterInfo> clusters(inds->size());

  for (size_t i = 0; i< inds->size(); i++)
  {
    cluster_helpers[i].inds = pcl::make_shared<pcl::PointIndices>(inds->at(i));
    for (int index : inds->at(i).indices)
    {
      cluster_helpers[i].centroid.add(pc->at(index));
    }
    cluster_helpers[i].centroid.get<pcl::PointXYZ>(clusters[i].center);

    cluster_helpers[i].feature_extractor.setInputCloud(pc);
    cluster_helpers[i].feature_extractor.setIndices(cluster_helpers[i].inds);
    cluster_helpers[i].feature_extractor.compute();
    pcl::PointXYZ min, max;
    cluster_helpers[i].feature_extractor.getAABB(min, max);
    clusters[i].volume_bbx = std::abs(max.x - min.x) * std::abs(max.y - min.y) * std::abs(max.z - min.z);

    cluster_helpers[i].hull.reset(new pcl::ConvexHull<pcl::PointXYZ>());
    cluster_helpers[i].hull_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cluster_helpers[i].hull->setDimension(3);
    cluster_helpers[i].hull->setComputeAreaVolume(true);
    cluster_helpers[i].hull->setInputCloud(pc);
    cluster_helpers[i].hull->setIndices(cluster_helpers[i].inds);
    cluster_helpers[i].hull->reconstruct(*(cluster_helpers[i].hull_cloud));
    clusters[i].volume = cluster_helpers[i].hull->getTotalVolume();

  }
  return clusters;
}

static std::vector<ClusterInfo> getClusterInfos(const pcl::PointCloud<pcl::PointXYZLNormal>::ConstPtr &cluster_pc)
{
  std::vector<ClusterInfoHelper<pcl::PointXYZLNormal>> cluster_helpers(cluster_pc->back().label + 1);

  for (size_t i = 0; i < cluster_pc->size(); i++)
  {
    uint32_t label = cluster_pc->at(i).label;
    if (cluster_helpers.size() <= label) cluster_helpers.resize(label + 1);
    cluster_helpers[label].inds->indices.push_back(static_cast<int>(i));
    cluster_helpers[label].centroid.add(cluster_pc->at(i));
  }


  for (auto it = cluster_helpers.begin(); it != cluster_helpers.end();)
  {
    if (it->inds->indices.size() < 3)
    {
      ROS_WARN_STREAM("Cluster too small (" << it->inds->indices.size() << " points)");
      it = cluster_helpers.erase(it);
      continue;
    }
    it++;
  }

  std::vector<ClusterInfo> clusters(cluster_helpers.size());
  for (size_t i=0; i < clusters.size(); i++)
  {
    cluster_helpers[i].centroid.get<pcl::PointXYZ>(clusters[i].center);

    cluster_helpers[i].feature_extractor.setInputCloud(cluster_pc);
    cluster_helpers[i].feature_extractor.setIndices(cluster_helpers[i].inds);
    cluster_helpers[i].feature_extractor.compute();
    pcl::PointXYZLNormal min, max;
    cluster_helpers[i].feature_extractor.getAABB(min, max);
    clusters[i].volume_bbx = std::abs(max.x - min.x) * std::abs(max.y - min.y) * std::abs(max.z - min.z);

    cluster_helpers[i].hull.reset(new pcl::ConvexHull<pcl::PointXYZLNormal>());
    cluster_helpers[i].hull_cloud.reset(new pcl::PointCloud<pcl::PointXYZLNormal>());
    cluster_helpers[i].hull->setDimension(3);
    cluster_helpers[i].hull->setComputeAreaVolume(true);
    cluster_helpers[i].hull->setInputCloud(cluster_pc);
    cluster_helpers[i].hull->setIndices(cluster_helpers[i].inds);
    cluster_helpers[i].hull->reconstruct(*(cluster_helpers[i].hull_cloud));
    clusters[i].volume = cluster_helpers[i].hull->getTotalVolume();
  }
  return clusters;
}

static std::vector<std::pair<size_t, size_t>> computePairs(const std::vector<ClusterInfo> &gt, const std::vector<ClusterInfo> &detected)
{
  // Build up matrix with all pairs
  boost::numeric::ublas::matrix<double> distances = boost::numeric::ublas::matrix<double>(gt.size(), detected.size());
  std::vector<std::pair<size_t, size_t>> indices;
  indices.reserve(gt.size() * detected.size());
  for (size_t i = 0; i < gt.size(); i++)
  {
    for (size_t j = 0; j < detected.size(); j++)
    {
      distances(i, j) = pcl::L2_Norm(gt[i].center.getArray3fMap(), detected[j].center.getArray3fMap(), 3);
      indices.push_back(std::make_pair(i, j));
    }
  }

  auto indicesComp = [&](const std::pair<size_t, size_t> &a, const std::pair<size_t, size_t> &b)
  {
    return distances(a.first, a.second) > distances(b.first, b.second);
  };

  boost::dynamic_bitset<> usedGtPoints(gt.size());
  boost::dynamic_bitset<> usedDetPoints(detected.size());

  std::vector<std::pair<size_t, size_t>> matchedPairs;

  matchedPairs.reserve(std::min(gt.size(), detected.size()));

  double MAX_DISTANCE = 0.2; // Maximal distance to be considered as same ROI

  for (std::make_heap(indices.begin(), indices.end(), indicesComp); !usedGtPoints.all() && !usedDetPoints.all(); std::pop_heap(indices.begin(), indices.end(), indicesComp), indices.pop_back())
  {
    const std::pair<size_t, size_t> &pair = indices.front();

    if (distances(pair.first, pair.second) > MAX_DISTANCE)
      break;

    if (usedGtPoints.test(pair.first) || usedDetPoints.test(pair.second))
      continue;

    matchedPairs.push_back(pair);
    usedGtPoints.set(pair.first);
    usedDetPoints.set(pair.second);
  }
  return matchedPairs;
}

struct ECEvalParams
{
  size_t detected_clusters = 0;
  double center_distance = 0;
  double volume_accuracy = 0;
  double volume_accuracy_bbx = 0;
  double volume_ratio = 0;
  double volume_ratio_bbx = 0;

  std::vector<double> distances;
  std::vector<double> volume_accuracies;
  std::vector<double> volume_accuracies_bbx;
  std::vector<double> volume_ratios;
  std::vector<double> volume_ratios_bbx;
};

class ExternalClusterEvaluator
{
private:
  std::shared_ptr<GtOctreeLoader> gtLoader;
  ros::Subscriber cluster_pointcloud_sub;
  ros::Subscriber superellipsoids_sub;
  ros::Publisher gt_superellipsoids_surface_pub;
  std::vector<ClusterInfo> gt_cluster_info;
  ECEvalParams current_params;
  boost::mutex current_params_mtx;

  void computeCurrentParams(const std::vector<ClusterInfo> &clusters);

public:
  ExternalClusterEvaluator(std::shared_ptr<GtOctreeLoader> gtLoader = nullptr, bool use_superellipsoids=true);

  void processReceivedClusters(const sensor_msgs::PointCloud2::ConstPtr &cluster_pc_msg);

  void processReceivedSuperellipsoids(const superellipsoid_msgs::SuperellipsoidArray::ConstPtr &se_arr_msg);

  ECEvalParams getCurrentParams();

  std::ostream& writeHeader(std::ostream &os);
  std::ostream& writeParams(std::ostream &os, const ECEvalParams &res);
};

} // namespace rvp_evaluation
