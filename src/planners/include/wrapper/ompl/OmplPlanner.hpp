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

    private:

        std::map <std::string, double> lower_limits_;
        std::map <std::string, double> upper_limits_;
        CartesianContraints cartesian_contraints_;
        bool hasCartesianConstraint_;
        base::samples::Joints start_joint_values_, goal_joint_values_;
        base::samples::RigidBodyState start_pose_, goal_pose_;

        //planner_type_ type_of_planner_;
        ompl_config::OmplConfig ompl_config_;
        int number_of_dimensions_;
        double max_solve_time_;
        unsigned int maxSteps_smoothing_ ;

        std::map<std::string, std::string> planner_specific_parameters_kv_;

        std::shared_ptr<ompl::geometric::PathGeometric> solution_path_ptr_;

        std::vector <std::map<std::string , double > >  path_joint_values_;
        std::vector <std::map<std::string , double > >  cartesian_path_;

        std::string chain_link_, tip_link_;

        bool setUpPlanningTaskInJointSpace(PlannerStatus &planner_status);
        bool setUpPlanningTaskInCartesianSpace(PlannerStatus &planner_status);	

        std::shared_ptr<RobotModel> robot_model_;		
        std::string planning_group_name_;	
        std::vector< std::pair<std::string,urdf::Joint> > planning_group_joints_;

        bool collisionCheckerStateValid(const ompl::base::State *state);
        void setUpPlanner (ompl::base::PlannerPtr &planner, ompl::base::SpaceInformationPtr &space_information, ompl::base::ProblemDefinitionPtr &problem_definition_ptr);
        bool solveProblem(ompl::base::PlannerPtr &planner,  const ompl::base::SpaceInformationPtr &space_information, const ompl::base::ProblemDefinitionPtr &problem_definition_ptr, 
                          base::JointsTrajectory &solution, PlannerStatus &planner_status);
        void setPlannerStatus(const ompl::base::PlannerStatus::StatusType &ompl_status, motion_planners::PlannerStatus &planner_status);
        bool checkStartState(const base::samples::Joints &current_robot_status, PlannerStatus &planner_status );
        bool solveTaskInCartesianSpace(base::JointsTrajectory &solution, PlannerStatus &planner_status);
        bool solveTaskInJointSpace(base::JointsTrajectory &solution, PlannerStatus &planner_status);
        bool cartesianSpaceStateValidityChecker(const ompl::base::State *state);    
        void setCartesianConstraints(CartesianContraints constraints);
        void simplifySolution(const ompl::base::ProblemDefinitionPtr &problem, ompl::geometric::PathSimplifier &path_simplifier, 
                              unsigned int step_size = 1, double duration = 0.0);

};
}// end namespace motion_planners
#endif // OMPLPLANNER_HPP
