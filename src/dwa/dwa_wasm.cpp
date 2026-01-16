#include <emscripten/bind.h>

#include "dwa_planner.h"

EMSCRIPTEN_BINDINGS(dwa_planner) {
  emscripten::class_<DWAPlanner>("DWAPlanner")
      .constructor<>()
      .function("plan", &DWAPlanner::plan)
      .function("sampleTrajectories", &DWAPlanner::sampleTrajectories)
      .function("bestTrajectory", &DWAPlanner::bestTrajectory)
      .function("bestCost", &DWAPlanner::bestCost)
      .function("setLimits", &DWAPlanner::setLimits)
      .function("setResolution", &DWAPlanner::setResolution)
      .function("setSamples", &DWAPlanner::setSamples)
      .function("setPredict", &DWAPlanner::setPredict)
      .function("setWeights", &DWAPlanner::setWeights)
      .function("setRobotRadius", &DWAPlanner::setRobotRadius)
      .function("setObsRange", &DWAPlanner::setObsRange);
  emscripten::register_vector<double>("VectorDouble");
}
