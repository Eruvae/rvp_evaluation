#include "rvp_evaluation/octree_provider_interfaces/saved_tree_interface.h"
#include <octomap/AbstractOcTree.h>

namespace rvp_evaluation
{

SavedTreeInterface::SavedTreeInterface(double tree_resolution) : planningTree(new octomap_vpp::RoiOcTree(tree_resolution))
{
}

bool SavedTreeInterface::readOctree(const std::string &filename)
{
  octomap_vpp::RoiOcTree *map = nullptr;
  octomap::AbstractOcTree *tree =  octomap::AbstractOcTree::read(filename);
  if (!tree)
    return false;

  map = dynamic_cast<octomap_vpp::RoiOcTree*>(tree);
  if(!map)
  {
    delete tree;
    return false;
  }
  tree_mtx.lock();
  planningTree.reset(map);
  planningTree->computeRoiKeys();
  tree_mtx.unlock();
  return true;
}

std::shared_ptr<octomap_vpp::RoiOcTree> SavedTreeInterface::getPlanningTree()
{
  return planningTree;
}

double SavedTreeInterface::getTreeResolution()
{
  return planningTree->getResolution();
}

MutexBase& SavedTreeInterface::getTreeMutex()
{
  return tree_mtx;
}

}
