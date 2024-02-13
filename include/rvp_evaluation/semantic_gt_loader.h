#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap_vpp/SemanticOcTree.h>
#include <gazebo_msgs/ModelState.h>

namespace rvp_evaluation
{

enum NodeClass
{
  UNKNOWN = 0,
  LEAF = 1,
  FRUIT = 2,
  STEM = 3
};

struct SemanticModelKeys
{
    octomap::KeySet stem_keys;
    octomap::KeySet leaf_keys;
    octomap::KeySet fruit_keys;
    octomap::KeySet peduncle_keys;
};

struct ModelInfo
{
  ModelInfo(const std::string &model, const gazebo_msgs::ModelState &state)
    : model(model), state(state) {}

  std::string model;
  gazebo_msgs::ModelState state;
};

class SemanticGtLoader
{
private:
  ros::NodeHandle nhp;
  double resolution;
  ros::Duration read_pose_timeout;
  std::vector<SemanticModelKeys> model_keys;
  std::unordered_map<std::string, size_t> model_name_map;
  std::vector<ModelInfo> model_list;
  std::unique_ptr<octomap_vpp::SemanticOcTree> tree;
  ros::Publisher tree_pub;
  
public:
  SemanticGtLoader(double resolution, const ros::Duration &read_pose_timeout=ros::Duration(1.0));
  void loadPlant(const std::string &name, const std::string &base_path);
  void readModelPoses();
  void insertSemanticKeys(const octomap::KeySet &keys, uint8_t class_id, const octomap::OcTreeKey &base_key, octomap::point3d &min_coord, octomap::point3d &max_coord);
  void updateGroundtruth(bool update_poses);
  void publishTree();
  uint8_t queryClass(const octomap::point3d &point);
};

} // namespace rvp_evaluation
