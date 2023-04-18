#include <ros/ros.h>
#include "rvp_evaluation/rvp_utils.h"
#include "rvp_evaluation/gt_octree_loader.h"
#include "rvp_evaluation/evaluator.h"
#include "rvp_evaluation/evaluator_external_clusters.h"

namespace rvp_evaluation
{

enum class EvalEpisodeEndParam
{
  TIME = 0,
  PLAN_DURATION = 1,
  PLAN_LENGTH = 2,
  NUM_EPEND_PARAMS = 3
};

enum class EvaluatorType
{
  NONE = 0b000,
  OCTREE_EVALUATOR_OLD = 0b001,
  OCTREE_EVALUATOR_NEW = 0b010,
  EXTERNAL_CLUSTER_EVALUATOR = 0b100,
  ALL = 0b111
};

ENUM_FLAG_OPERATORS(EvaluatorType)

class EvaluationManager
{
private:
  std::shared_ptr<rvp_evaluation::GtOctreeLoader> gtLoader;
  std::unique_ptr<rvp_evaluation::Evaluator> evaluator;
  std::unique_ptr<rvp_evaluation::ExternalClusterEvaluator> external_cluster_evaluator;

  EvaluatorType active_evaluators;

  // Evaluator variables
  size_t eval_trial_num;
  std::ofstream eval_resultsFile;
  std::ofstream eval_resultsFileOld;
  std::ofstream eval_externalClusterFile;
  std::ofstream eval_fruitCellPercFile;
  std::ofstream eval_volumeAccuracyFile;
  std::ofstream eval_distanceFile;
  ros::Time eval_plannerStartTime;
  double eval_passedTime;
  double eval_accumulatedPlanDuration;
  double eval_accumulatedPlanLength;
  std::string eval_lastStep;

  void setEvaluatorStartParams();

public:
  EvaluationManager(EvaluatorType active_evaluators, double groundtruth_resolution, std::shared_ptr<OctreeProviderInterface> tree_interface);

  bool startEvaluator(size_t numEvals, EvalEpisodeEndParam episodeEndParam, double episodeDuration, int start_index,
                      bool randomize_plants, const octomap::point3d &min, const octomap::point3d &max, double min_dist,
                      bool with_trolley);

  bool saveEvaluatorData(double plan_length, double traj_duration, size_t segment, double trolley_pos, double trolley_height);
  bool resetEvaluator();
};

} // namespace rvp_evaluation