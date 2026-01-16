#ifndef __MPC_PLANNER_GENERATED_MODULES_H__
#define __MPC_PLANNER_GENERATED_MODULES_H__

#include <mpc_planner_modules/mpc_base.h>
#include <mpc_planner_modules/contouring.h>
#include <mpc_planner_modules/guidance_constraints.h>
#include <mpc_planner_modules/linearized_constraints.h>
#include <mpc_planner_modules/ellipsoid_constraints.h>

namespace MPCPlanner
{
	class Solver;
	inline void initializeModules(std::vector<std::shared_ptr<ControllerModule>> &modules, std::shared_ptr<Solver> solver)
	{
		modules.emplace_back(nullptr);
		modules.back() = std::make_shared<MPCBaseModule>(solver);
		modules.emplace_back(nullptr);
		modules.back() = std::make_shared<Contouring>(solver);
		modules.emplace_back(nullptr);
		modules.back() = std::make_shared<GuidanceConstraints>(solver);

	}
}
#endif