#include "rvp_evaluation/octree_provider_interfaces/provided_tree_interface.h"

namespace rvp_evaluation
{

ProvidedTreeInterface::ProvidedTreeInterface(std::shared_ptr<octomap_vpp::RoiOcTree> planningTree)
  : planningTree(planningTree), tree_mtx(own_mtx)
{
}

ProvidedTreeInterface::ProvidedTreeInterface(std::shared_ptr<octomap_vpp::RoiOcTree> planningTree, boost::mutex &tree_mtx)
  : planningTree(planningTree), tree_mtx(tree_mtx)
{
}

std::shared_ptr<octomap_vpp::RoiOcTree> ProvidedTreeInterface::getPlanningTree()
{
  return planningTree;
}

double ProvidedTreeInterface::getTreeResolution()
{
  return planningTree->getResolution();
}

boost::mutex& ProvidedTreeInterface::getTreeMutex()
{
  return tree_mtx;
}

}
