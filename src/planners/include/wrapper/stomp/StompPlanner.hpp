
#ifndef STOMPPLANNER_HPP_
#define STOMPPLANNER_HPP_

#include <abstract/AbstractPlanner.hpp>
#include "OptimizationTask.hpp"
#include "HandleStompConfig.hpp"


namespace motion_planners
{
  

class StompPlanner: public motion_planners::AbstractPlanner
{
	public:
		StompPlanner();

		~StompPlanner();
	
		bool initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string config_file_path);
	
		bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status);
		
		void updateInitialTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal, PlannerStatus &planner_status);

	
	private:	
		boost::shared_ptr<stomp::Stomp> stomp_;
		std::shared_ptr<RobotModel> robot_model_;
		stomp::StompConfig stomp_config_;
		stomp::DebugConfig debug_config_;
		std::shared_ptr<OptimizationTask> optimization_task_;
		stomp::CovariantMovementPrimitive tmp_policy;

		std::string planning_group_name_;
		std::vector< std::string> planning_group_joints_name_;
	
};
}// end planner

#endif
