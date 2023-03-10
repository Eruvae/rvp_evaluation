#include <ros/ros.h>
#include "rvp_evaluation/evaluator_external_clusters.h"
#include "std_srvs/Empty.h"
#include "roi_viewpoint_planner_msgs/StartEvaluator.h"
#include "roi_viewpoint_planner_msgs/SaveEvaluatorData.h"

using namespace rvp_evaluation;

std::unique_ptr<ExternalClusterEvaluator> evaluator;

// Evaluator variables
size_t eval_trial_num;
std::ofstream eval_externalClusterFile;
ros::Time eval_plannerStartTime;
double eval_passedTime;
double eval_accumulatedPlanDuration;
double eval_accumulatedPlanLength;

void setEvaluatorStartParams()
{
  std::string file_index_str = std::to_string(eval_trial_num);
  eval_externalClusterFile = std::ofstream("planner_results_ec" + file_index_str + ".csv");
  eval_externalClusterFile << "Time (s),Plan duration (s),Plan Length,";
  evaluator->writeHeader(eval_externalClusterFile) << std::endl;
  eval_plannerStartTime = ros::Time::now();
  eval_passedTime = 0;
  eval_accumulatedPlanDuration = 0;
  eval_accumulatedPlanLength = 0;
}

bool startEvaluator(roi_viewpoint_planner_msgs::StartEvaluator::Request &req, roi_viewpoint_planner_msgs::StartEvaluator::Response &res)
{
  eval_trial_num = req.starting_index;
  setEvaluatorStartParams();
  res.success = true;
  return true;
}

bool saveEvaluatorData(roi_viewpoint_planner_msgs::SaveEvaluatorData::Request &req, roi_viewpoint_planner_msgs::SaveEvaluatorData::Response &res)
{
  ros::Time currentTime = ros::Time::now();

  eval_passedTime = (currentTime - eval_plannerStartTime).toSec();
  eval_accumulatedPlanDuration += req.traj_duration;
  eval_accumulatedPlanLength += req.plan_length;

  eval_externalClusterFile << eval_passedTime << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
  evaluator->writeParams(eval_externalClusterFile, evaluator->getCurrentParams()) << std::endl;

  return true;
}

bool resetEvaluator(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  eval_externalClusterFile.close();
  eval_trial_num++;
  setEvaluatorStartParams();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_external_clusters");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ros::ServiceServer startEvaluatorService = nhp.advertiseService("start_evaluator", startEvaluator);
  ros::ServiceServer saveEvaluatorDataService = nhp.advertiseService("save_evaluator_data", saveEvaluatorData);
  ros::ServiceServer resetEvaluatorService = nhp.advertiseService("reset_evaluator", resetEvaluator);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  evaluator.reset(new ExternalClusterEvaluator());
  ros::waitForShutdown();
}
