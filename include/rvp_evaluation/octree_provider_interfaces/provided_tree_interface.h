#ifndef PROVIDED_TREE_INTERFACE_H
#define PROVIDED_TREE_INTERFACE_H

#include "rvp_evaluation/octree_provider_interface.h"

namespace rvp_evaluation
{

class ProvidedTreeInterface : public OctreeProviderInterface
{
private:
  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;
  boost::mutex &tree_mtx;
  boost::mutex own_mtx; // Used only if an external one isn't specified

public:
  ProvidedTreeInterface(std::shared_ptr<octomap_vpp::RoiOcTree> planningTree);
  ProvidedTreeInterface(std::shared_ptr<octomap_vpp::RoiOcTree> planningTree, boost::mutex &tree_mtx);

  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree();
  virtual double getTreeResolution();
  virtual boost::mutex& getTreeMutex();
};

}


#endif // PROVIDED_TREE_INTERFACE_H
