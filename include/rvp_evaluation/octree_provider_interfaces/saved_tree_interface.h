#ifndef SAVED_TREE_INTERFACE_H
#define SAVED_TREE_INTERFACE_H

#include "rvp_evaluation/octree_provider_interface.h"

namespace rvp_evaluation
{

class SavedTreeInterface : public OctreeProviderInterface
{
private:
  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;
  boost::mutex tree_mtx;

public:
  SavedTreeInterface(double tree_resolution);
  bool readOctree(const std::string &filename);

  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree();
  virtual double getTreeResolution();
  virtual boost::mutex& getTreeMutex();
};

}

#endif // SAVED_TREE_INTERFACE_H
