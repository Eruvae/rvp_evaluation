#include "rvp_evaluation/semantic_gt_loader.h"

#include "rvp_evaluation/o3d_utils.h"
#include "rvp_evaluation/rvp_utils.h"

#include <octomap_msgs/Octomap.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <boost/algorithm/string.hpp>

#include <filesystem>

namespace rvp_evaluation
{

SemanticGtLoader::SemanticGtLoader(double resolution) : nhp("~"), resolution(resolution), tree(std::make_unique<octomap_vpp::SemanticOcTree>(resolution))
{
  tree_pub = nhp.advertise<octomap_msgs::Octomap>("semantic_gt", 1, true);

  std::string plant_model_path = ros::package::getPath("ur_with_cam_gazebo");

  loadPlant("VG07_1", plant_model_path + "/models/capsicum_plant_1/meshes/VG07_1.dae");
  loadPlant("VG07_2", plant_model_path + "/models/capsicum_plant_2/meshes/VG07_2.dae");
  loadPlant("VG07_3", plant_model_path + "/models/capsicum_plant_3/meshes/VG07_3.dae");
  loadPlant("VG07_4", plant_model_path + "/models/capsicum_plant_4/meshes/VG07_4.dae");
  loadPlant("VG07_5", plant_model_path + "/models/capsicum_plant_5/meshes/VG07_5.dae");
  loadPlant("VG07_6", plant_model_path + "/models/capsicum_plant_6/meshes/VG07_6.dae");
  loadPlant("VG07_6_more_occ", plant_model_path + "/models/capsicum_plant_6_more_occ/meshes/VG07_6_more_occ.dae");
  loadPlant("VG07_6_no_fruits", plant_model_path + "/models/capsicum_plant_6_no_fruits/meshes/VG07_6_no_fruits.dae");
  loadPlant("VG07_6_one_fruit", plant_model_path + "/models/capsicum_plant_6_one_fruit/meshes/VG07_6_one_fruit.dae");
  loadPlant("VG07_7", plant_model_path + "/models/capsicum_plant_7/meshes/VG07_7.dae");
  loadPlant("VG07_8", plant_model_path + "/models/capsicum_plant_8/meshes/VG07_8.dae");
  loadPlant("VG07_9", plant_model_path + "/models/capsicum_plant_9/meshes/VG07_9.dae");

  updateGroundtruth(true);
}

void SemanticGtLoader::loadPlant(const std::string &name, const std::string &base_path)
{
  ROS_INFO_STREAM("Loading plant " << name);
  if (!std::filesystem::exists(base_path))
  {
    ROS_ERROR_STREAM("Plant mesh base path for " << name << " does not exist");
    return;
  }
  std::string fruit_path = base_path;
  fruit_path.insert(base_path.length() - 4, "_fruitonly");
  std::string leaf_path = base_path;
  leaf_path.insert(base_path.length() - 4, "_leaves");
  std::string stem_path = base_path;
  stem_path.insert(base_path.length() - 4, "_stem");
  
  std::shared_ptr<open3d::geometry::VoxelGrid> plant_grid = readModelO3D(base_path, resolution);
  if (!plant_grid)
  {
    ROS_ERROR_STREAM("Could not load plant model " << name);
    return;
  }
  std::shared_ptr<open3d::geometry::VoxelGrid> fruit_grid = nullptr;
  if (std::filesystem::exists(fruit_path))
  {
    fruit_grid = readModelO3D(fruit_path, resolution);
  }
  std::shared_ptr<open3d::geometry::VoxelGrid> leaf_grid = nullptr;
  if (std::filesystem::exists(leaf_path))
  {
    leaf_grid = readModelO3D(leaf_path, resolution);
  }
  std::shared_ptr<open3d::geometry::VoxelGrid> stem_grid = nullptr;
  if (std::filesystem::exists(stem_path))
  {
    stem_grid = readModelO3D(stem_path, resolution);
  }

  ROS_INFO_STREAM("Files loaded");

  SemanticModelKeys keys;
  for (const auto &voxel : plant_grid->GetVoxels())
  {
    const Eigen::Vector3d vc = plant_grid->GetVoxelCenterCoordinate(voxel.grid_index_);
    bool is_fruit = fruit_grid ? fruit_grid->CheckIfIncluded({vc})[0] : false;
    bool is_leaf = leaf_grid ? leaf_grid->CheckIfIncluded({vc})[0] : false;
    bool is_stem = stem_grid ? stem_grid->CheckIfIncluded({vc})[0] : false;

    octomap::OcTreeKey key = tree->coordToKey(octomap::point3d(static_cast<float>(vc(0)), static_cast<float>(-vc(2)), static_cast<float>(vc(1))));
    if (is_fruit)
        keys.fruit_keys.insert(key);
    else if (is_leaf)
        keys.leaf_keys.insert(key);
    else if (is_stem)
        keys.stem_keys.insert(key);
    
  }

  ROS_INFO_STREAM("Plant contains " << keys.leaf_keys.size() << " leaf keys, " << keys.fruit_keys.size() << " fruit keys, and " << keys.stem_keys.size() << " stem keys");

  model_keys.push_back(keys);
  model_name_map.insert(std::make_pair(name, model_keys.size() - 1));
}

void SemanticGtLoader::readModelPoses()
{
  gazebo_msgs::ModelStatesConstPtr model_states = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states", ros::Duration(1.0));
  if (!model_states)
  {
    ROS_ERROR_STREAM("Model states message not received; could not read plant poses");
    return;
  }
  model_list.clear();
  for (size_t i=0; i < model_states->name.size(); i++)
  {
    gazebo_msgs::ModelState state;
    state.model_name = model_states->name[i];
    state.pose = model_states->pose[i];
    state.twist = model_states->twist[i];
    if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_6_more_occ"))
    {
      model_list.push_back(ModelInfo("VG07_6_more_occ", state));
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_6_no_fruits"))
    {
      model_list.push_back(ModelInfo("VG07_6_no_fruits", state));
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_6_one_fruit"))
    {
      model_list.push_back(ModelInfo("VG07_6_one_fruit", state));
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_6"))
    {
      model_list.push_back(ModelInfo("VG07_6", state));
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_3"))
    {
      model_list.push_back(ModelInfo("VG07_3", state));
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_4"))
    {
      model_list.push_back(ModelInfo("VG07_4", state));
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_5"))
    {
      model_list.push_back(ModelInfo("VG07_5", state));
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_7"))
    {
      model_list.push_back(ModelInfo("VG07_7", state));
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_8"))
    {
      model_list.push_back(ModelInfo("VG07_8", state));
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_9"))
    {
      model_list.push_back(ModelInfo("VG07_9", state));
    }
  }
}

void SemanticGtLoader::insertSemanticKeys(const octomap::KeySet &keys, uint8_t class_id, const octomap::OcTreeKey &base_key, octomap::point3d &min_coord, octomap::point3d &max_coord)
{
  const octomap::OcTreeKey ZERO_KEY = tree->coordToKey(0, 0, 0);
  for (const octomap::OcTreeKey &key : keys)
  {
    octomap::OcTreeKey resKey = addKeys(base_key, key, ZERO_KEY);
    octomap::point3d coord = tree->keyToCoord(resKey);

    auto setMinMax = [](const float &c, float &min, float &max) -> void
    {
      if (c < min) min = c;
      if (c > max) max = c;
    };
    setMinMax(coord.x(), min_coord.x(), max_coord.x());
    setMinMax(coord.y(), min_coord.y(), max_coord.y());
    setMinMax(coord.z(), min_coord.z(), max_coord.z());

    octomap_vpp::SemanticOcTreeNode *node = tree->setNodeValue(resKey, tree->getClampingThresMaxLog());
    node->setNodeClass(class_id);
  }
}

void SemanticGtLoader::updateGroundtruth(bool update_poses)
{
  if (update_poses)
    readModelPoses();

  tree->clear();

  
  octomap::point3d min_coord(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  octomap::point3d max_coord(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());

  for (const ModelInfo &plant : model_list)
  {
    const std::string &model = plant.model;
    octomap::point3d loc(plant.state.pose.position.x, plant.state.pose.position.y, plant.state.pose.position.z);

    auto it = model_name_map.find(model);
    if (it == model_name_map.end())
    {
      continue; // Unknown models are skipped
    }

    const SemanticModelKeys &keys = model_keys[it->second];
    octomap::OcTreeKey base_key = tree->coordToKey(loc);
    insertSemanticKeys(keys.leaf_keys, 0, base_key, min_coord, max_coord);
    insertSemanticKeys(keys.fruit_keys, 1, base_key, min_coord, max_coord);
    insertSemanticKeys(keys.stem_keys, 2, base_key, min_coord, max_coord);
  }

  publishTree();
}

void SemanticGtLoader::publishTree()
  {
    if (!tree)
    {
      ROS_ERROR_STREAM("No tree to publish");
      return;
    }
    octomap_msgs::Octomap octomap_msg;
    octomap_msgs::fullMapToMsg(*tree, octomap_msg);
    octomap_msg.header.frame_id = "world";
    octomap_msg.header.stamp = ros::Time::now();
    tree_pub.publish(octomap_msg);
  }

} // namespace rvp_evaluation