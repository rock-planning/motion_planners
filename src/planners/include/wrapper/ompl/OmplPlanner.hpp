#ifndef OMPLPLANNER_HPP
#define OMPLPLANNER_HPP

#include <vector>
#include <string>


#include <urdf_model/model.h>


#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/sbl/SBL.h> //SBL
#include <ompl/geometric/planners/est/EST.h> //EST
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h> //LBKPIECE
#include <ompl/geometric/planners/kpiece/BKPIECE1.h> //BKPIECE
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h> //RRT
#include <ompl/geometric/planners/rrt/RRTConnect.h> //RRTConnect
#include <ompl/geometric/planners/rrt/RRTstar.h> //RRTstar
#include <ompl/geometric/planners/prm/PRM.h> // PRM
#include <ompl/geometric/planners/prm/PRMstar.h> //PRMstar

#include <base-logging/Logging.hpp>
#include <abstract/AbstractPlanner.hpp>
#include <RobotModel.hpp>
#include "RobotFreeJointParameter.hpp"
#include "OmplConfig.hpp"
#include "HandleOmplConfig.hpp"


namespace motion_planners
{
    
class OmplPlanner: public motion_planners::AbstractPlanner
{
    public:
	OmplPlanner();
	~OmplPlanner();
	bool initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string config_file_path);
	bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status);		
	void updateInitialTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal, PlannerStatus &planner_status);
	
	bool collisionCheckerStateValid(const ompl::base::State *state);
	void setPanningGroupName(const std::string  &planningGroupName);
	
	//MotionPlanningResponse getMotionPlanningResponse();
	bool solveTaskInCartesianSpace(base::JointsTrajectory &solution, PlannerStatus &planner_status);
	bool solveTaskInJointSpace(base::JointsTrajectory &solution, PlannerStatus &planner_status);
	bool cartesianSpaceStateValidityChecker(const ompl::base::State *state);    
	void setPlannerStatus(const ompl::base::PlannerStatus::StatusType &ompl_status, motion_planners::PlannerStatus &planner_status);
	
	bool checkStartState(const base::samples::Joints &current_robot_status, PlannerStatus &planner_status );
	
	void motionPlanningRequest( const base::samples::Joints &current_robot_status,
				    const base::samples::RigidBodyState &goal_pose_in_chain_root, 
				    std::vector<RobotFreeJointParameter>  robot_free_joints_parameters, 
				    PlannerStatus &planner_status);
	void motionPlanningRequest( const base::samples::Joints &current_robot_status,
				    const base::samples::Joints &target_joint_status,                                                
				    PlannerStatus &planner_status);
    
	void setCartesianConstraints(CartesianContraints constraints);
    
    private:

	std::map <std::string, double> lower_limits_;
	std::map <std::string, double> upper_limits_;
	CartesianContraints cartesian_contraints_;
	bool hasCartesianConstraint_;
	
	//std::vector< std::pair<std::string , urdf::Joint >  > planning_groups_joints;

	//std::map <std::string, double> start_joint_values_;
	//std::map <std::string, double> goal_joint_values_;
	base::samples::Joints start_joint_values_, goal_joint_values_;
	
	base::samples::RigidBodyState start_pose_, goal_pose_;
	    
	//planner_type_ type_of_planner_;
	int number_of_dimensions_;
	double max_solve_time_;
	unsigned int maxSteps_smoothing_ ;

	std::map<std::string, std::string> planner_specific_parameters_kv_;

	#if OMPL_VERSION_VALUE < 1001000
	    boost::shared_ptr<ompl::geometric::PathGeometric> solution_path_ptr_;
	#else
	    std::shared_ptr<ompl::geometric::PathGeometric> solution_path_ptr_;
	#endif


	//MotionPlanningRequest motion_planning_request;

	std::vector <std::map<std::string , double > >  path_joint_values_;
	std::vector <std::map<std::string , double > >  cartesian_path_;


	std::string base_link_, chain_link_, tip_link_;


	bool setUpPlanningTaskInJointSpace(PlannerStatus &planner_status);
	bool setUpPlanningTaskInCartesianSpace(PlannerStatus &planner_status);	

	//MotionPlanningResponse motion_planning_response;


		
	std::shared_ptr<RobotModel> robot_model_;		
	std::string planning_group_name_;	
	std::vector< std::pair<std::string,urdf::Joint> > planning_group_joints_;


	typedef std::map<std::string, double> ompl_state_corresponding_joint_values_type;
	typedef std::map<std::string , ompl_state_corresponding_joint_values_type> ompl_state_joint_values_type;
	ompl_state_joint_values_type ompl_states_joint_values;
	
	ompl_config::OmplConfig ompl_config_;


	protected:
	void solution_path_ptr();
};
}// end namespace motion_planners
#endif // OMPLPLANNER_HPP
