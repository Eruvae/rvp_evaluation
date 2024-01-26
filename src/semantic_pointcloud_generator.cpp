#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_vpp/octomap_pcl.h>
#include "rvp_evaluation/semantic_gt_loader.h"

std::unique_ptr<rvp_evaluation::SemanticGtLoader> semantic_gt_loader;
std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;
std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub;
std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> transform_filter;

ros::Publisher semantic_pc_pub;

std::string map_frame;

void processPointcloud(const sensor_msgs::PointCloud2ConstPtr &pc)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pc, *pcl_cloud);

  geometry_msgs::TransformStamped pcFrameTf;
  try
  {
    pcFrameTf = tf_buffer->lookupTransform(map_frame, pc->header.frame_id, pc->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZL>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZL>);
  out_cloud->header = pcl_cloud->header;
  out_cloud->points.reserve(pcl_cloud->points.size());

  for (auto &p : pcl_cloud->points)
  {
    
    octomap::point3d point  = octomap_vpp::pclPointToOctomap(p);
    tf2::doTransform(point, point, pcFrameTf);
    uint8_t node_class = semantic_gt_loader->queryClass(point);
    pcl::PointXYZL pl;
    pl.x = p.x; pl.y = p.y; pl.z = p.z;
    pl.label = node_class;
    out_cloud->points.push_back(pl);
  }

  semantic_pc_pub.publish(out_cloud);  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "semantic_pointcloud_generator");
  ros::NodeHandle nhp("~");

  double res = nhp.param<double>("resolution", 0.01);
  std::string input_pc = nhp.param<std::string>("input_pc", "/camera/depth/points");
  map_frame = nhp.param<std::string>("map_frame", "world");

  semantic_pc_pub = nhp.advertise<pcl::PointCloud<pcl::PointXYZL>>("semantic_pc", 1);
  semantic_gt_loader.reset(new rvp_evaluation::SemanticGtLoader(res));
  tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer, nhp));
  pc_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nhp, input_pc, 100));
  transform_filter.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*pc_sub, *tf_buffer, map_frame, 100, nhp));

  transform_filter->registerCallback(processPointcloud);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
}

