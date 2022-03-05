#include <ros/ros.h>
#include "rvp_evaluation/evaluator_external_clusters.h"

using namespace rvp_evaluation;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_external_clusters");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ExternalClusterEvaluator evaluator;
  ros::waitForShutdown();
}
