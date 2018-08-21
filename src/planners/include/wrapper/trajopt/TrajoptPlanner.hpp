
#ifndef TRAJOPTPLANNER_HPP_
#define TRAJOPTPLANNER_HPP_

#include <abstract/AbstractPlanner.hpp>
#include <RobotModel.hpp>
#include "json/json.h"
#include "trajopt/problem_description.hpp"
#include "sco/optimizers.hpp"
#include "utils/clock.hpp"

using namespace trajopt;
using namespace Json;

namespace motion_planners
{
  

class TrajoptPlanner: public motion_planners::AbstractPlanner
{
	public:
                TrajoptPlanner();

                ~TrajoptPlanner();
	
		bool initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string config_file_path);

                bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status);

                void updateInitialTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal, PlannerStatus &planner_status);

private:
                std::shared_ptr<RobotModel> m_robot_model_;

                std::string m_planning_group_name_;
                std::vector< std::string> m_planning_group_joints_name_;

                TrajOptProbPtr m_prb;
                BasicTrustRegionSQP m_opt;
	
};
}// end planner

#endif
