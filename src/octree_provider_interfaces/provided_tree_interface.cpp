#include "rvp_evaluation/octree_provider_interfaces/provided_tree_interface.h"

namespace rvp_evaluation
{

std::shared_ptr<octomap_vpp::RoiOcTree> ProvidedTreeInterface::getPlanningTree()
{
  return planningTree;
}

double ProvidedTreeInterface::getTreeResolution()
{
  return planningTree->getResolution();
}

MutexBase& ProvidedTreeInterface::getTreeMutex()
{
  return *tree_mtx;
}

}
