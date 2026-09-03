#ifndef LIOM_LOCAL_PLANNER_LIOM_LOCAL_PLANNER_H
#define LIOM_LOCAL_PLANNER_LIOM_LOCAL_PLANNER_H

#include <memory>
#include "liom_local_planner/optimizer_interface.h"
#include "liom_local_planner/planner_config.h"
#include "liom_local_planner/environment.h"
#include "liom_local_planner/coarse_path_planner.h"
#include "liom_local_planner/path_planner.h"

namespace liom_local_planner {

namespace math = common::math;

class LiomLocalPlanner {
public:
  explicit LiomLocalPlanner(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env);

  void set_global_path(const std::vector<math::Pose> &path) {
    global_path_ = path;
  }

  bool Plan(const FullStates &prev_solution, const TrajectoryPoint &start, const TrajectoryPoint &goal, FullStates &result);

public:
  std::shared_ptr<PlannerConfig> config_;
  std::shared_ptr<Environment> env_;
  std::shared_ptr<IOptimizer> problem_;

  std::vector<math::Pose> global_path_;

  // CoarsePathPlanner coarse_path_planner_;
  PathPlanner coarse_path_planner_;

  bool CheckGuessFeasibility(const FullStates &guess);

  FullStates StitchPreviousSolution(const FullStates &solution, const TrajectoryPoint &start);

  FullStates GenerateGuessFromPath(const std::vector<math::Pose> &path, const TrajectoryPoint &start);

  FullStates ResamplePath(const std::vector<math::Pose> &path) const;

  std::vector<double> GenerateOptimalTimeProfileSegment(const std::vector<double> &stations, double start_time) const;
};

}

#endif  // LIOM_LOCAL_PLANNER_LIOM_LOCAL_PLANNER_H_