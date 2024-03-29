#ifndef PLANNER_INTERFACE_H
#define PLANNER_INTERFACE_H

#include <octomap_vpp/RoiOcTree.h>
#include <boost/thread/mutex.hpp>

namespace rvp_evaluation
{

class OctreeProviderInterface
{
public:
  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree() = 0;
  virtual double getTreeResolution() = 0;
  virtual boost::mutex& getTreeMutex() = 0;
};

}

#endif // PLANNER_INTERFACE_H
