#pragma once

#include "rvp_evaluation/octree_provider_interface.h"

namespace rvp_evaluation
{

class ProvidedTreeInterface : public OctreeProviderInterface
{
private:
  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;
  std::unique_ptr<MutexBase> tree_mtx;

public:
  ProvidedTreeInterface(std::shared_ptr<octomap_vpp::RoiOcTree> planningTree)
    : planningTree(planningTree), tree_mtx(new MutexRef<boost::mutex>()) {}

  template <typename T>
  ProvidedTreeInterface(std::shared_ptr<octomap_vpp::RoiOcTree> planningTree, T &tree_mtx)
    : planningTree(planningTree), tree_mtx(new MutexRef<T>(tree_mtx)) {}

  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree();
  virtual double getTreeResolution();
  virtual MutexBase& getTreeMutex();
};

} // namespace rvp_evaluation
