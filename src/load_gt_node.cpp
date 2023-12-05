#include <ros/ros.h>
#include "rvp_evaluation/gt_octree_loader.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_gt");
  ros::NodeHandle nhp("~");

  double res = nhp.param<double>("resolution", 0.01);

  rvp_evaluation::GtOctreeLoader gtLoader(res);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
}
