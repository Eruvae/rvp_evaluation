#include "rvp_evaluation/evaluation_manager.h"

namespace rvp_evaluation
{

EvaluationManager::EvaluationManager(EvaluatorType active_evaluators, double groundtruth_resolution, std::shared_ptr<OctreeProviderInterface> tree_interface)
    : active_evaluators(active_evaluators)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_eval("evaluator");

  gtLoader.reset(new rvp_evaluation::GtOctreeLoader(groundtruth_resolution));

  if (test(active_evaluators, EvaluatorType::OCTREE_EVALUATOR_OLD) || test(active_evaluators, EvaluatorType::OCTREE_EVALUATOR_NEW))
  {
    evaluator.reset(new rvp_evaluation::Evaluator(tree_interface, nh, nh_eval, true, false, gtLoader));
    evaluator->saveGtAsColoredCloud();
  }
  if (test(active_evaluators, EvaluatorType::EXTERNAL_CLUSTER_EVALUATOR))
  {
    external_cluster_evaluator.reset(new rvp_evaluation::ExternalClusterEvaluator(gtLoader));
  }
  eval_trial_num = 0;
}

bool EvaluationManager::startEvaluator(size_t numEvals, EvalEpisodeEndParam episodeEndParam, double episodeDuration, int start_index,
                                   bool randomize_plants, const octomap::point3d &min, const octomap::point3d &max, double min_dist,
                                   bool with_trolley)
{
  eval_trial_num = 0;
  setEvaluatorStartParams();
  return true;
}

void EvaluationManager::setEvaluatorStartParams()
{
  std::string file_index_str = std::to_string(eval_trial_num);

  if (test(active_evaluators, EvaluatorType::OCTREE_EVALUATOR_OLD))
  {
    eval_resultsFileOld = std::ofstream("planner_results_old" + file_index_str + ".csv");

    eval_resultsFileOld << "Time (s),Plan duration (s),Plan Length,";
    evaluator->writeHeaderOld(eval_resultsFileOld) << ",Step,Segment,Trolley Position,Trolley Height" << std::endl;
  }

  if (test(active_evaluators, EvaluatorType::OCTREE_EVALUATOR_NEW))
  {
    eval_resultsFile = std::ofstream("planner_results_" + file_index_str + ".csv");
    eval_fruitCellPercFile = std::ofstream("results_fruit_cells_" + file_index_str + ".csv");
    eval_volumeAccuracyFile = std::ofstream("results_volume_accuracy_" + file_index_str + ".csv");
    eval_distanceFile = std::ofstream("results_distances_" + file_index_str + ".csv");

    eval_resultsFile << "Time (s),Plan duration (s),Plan Length,";
    evaluator->writeHeader(eval_resultsFile) << ",Step,Segment,Trolley Position,Trolley Height" << std::endl;
  }

  if (test(active_evaluators, EvaluatorType::EXTERNAL_CLUSTER_EVALUATOR))
  {
    eval_externalClusterFile = std::ofstream("planner_results_ec" + file_index_str + ".csv");

    eval_externalClusterFile << "Time (s),Plan duration (s),Plan Length,";
    external_cluster_evaluator->writeHeader(eval_externalClusterFile)<< ",Step,Segment,Trolley Position,Trolley Height" << std::endl;
  }

  eval_plannerStartTime = ros::Time::now();
  eval_passedTime = 0;
  eval_accumulatedPlanDuration = 0;
  eval_accumulatedPlanLength = 0;
}

bool EvaluationManager::saveEvaluatorData(double plan_length, double traj_duration, size_t segment, double trolley_pos, double trolley_height)
{
  ros::Time currentTime = ros::Time::now();

  eval_passedTime = (currentTime - eval_plannerStartTime).toSec();

  eval_accumulatedPlanDuration += traj_duration;
  eval_accumulatedPlanLength += plan_length;

  if (test(active_evaluators, EvaluatorType::OCTREE_EVALUATOR_OLD))
  {
    rvp_evaluation::EvaluationParametersOld resOld = evaluator->processDetectedRoisOld();
    eval_resultsFileOld << eval_passedTime << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
    evaluator->writeParamsOld(eval_resultsFileOld, resOld) << "," << eval_lastStep << "," << segment << "," << trolley_pos << "," << trolley_height << std::endl;
  }

  if (test(active_evaluators, EvaluatorType::OCTREE_EVALUATOR_NEW))
  {
    rvp_evaluation::EvaluationParameters res = evaluator->processDetectedRois(true, eval_trial_num, static_cast<size_t>(eval_passedTime));
    eval_resultsFile << eval_passedTime << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
    evaluator->writeParams(eval_resultsFile, res) << "," << eval_lastStep << "," << segment << "," << trolley_pos << "," << trolley_height << std::endl;
    writeVector(eval_fruitCellPercFile, eval_passedTime, res.fruit_cell_percentages) << std::endl;
    writeVector(eval_volumeAccuracyFile, eval_passedTime, res.volume_accuracies) << std::endl;
    writeVector(eval_distanceFile, eval_passedTime, res.distances) << std::endl;
  }

  if (test(active_evaluators, EvaluatorType::EXTERNAL_CLUSTER_EVALUATOR))
  {
    eval_externalClusterFile << eval_passedTime << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
    external_cluster_evaluator->writeParams(eval_externalClusterFile, external_cluster_evaluator->getCurrentParams()) << "," << eval_lastStep << "," << segment << "," << trolley_pos << "," << trolley_height << std::endl;
  }

  return true;
}

bool EvaluationManager::resetEvaluator()
{
  if (test(active_evaluators, EvaluatorType::OCTREE_EVALUATOR_OLD))
  {
    eval_resultsFileOld.close();
  }
  if (test(active_evaluators, EvaluatorType::OCTREE_EVALUATOR_NEW))
  {
    eval_resultsFile.close();
    eval_fruitCellPercFile.close();
    eval_volumeAccuracyFile.close();
    eval_distanceFile.close();
  }
  if (test(active_evaluators, EvaluatorType::EXTERNAL_CLUSTER_EVALUATOR))
  {
    eval_externalClusterFile.close();
  }

  eval_trial_num++;
  setEvaluatorStartParams();
  return true;
}


} // namespace rvp_evaluation