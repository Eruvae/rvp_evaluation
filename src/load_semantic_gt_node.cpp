#include <ros/ros.h>
#include <ros/package.h>
#include "rvp_evaluation/semantic_gt_loader.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_gt");
  ros::NodeHandle nhp("~");

  double res = nhp.param<double>("resolution", 0.01);

  rvp_evaluation::SemanticGtLoader gtLoader(res);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();
}

