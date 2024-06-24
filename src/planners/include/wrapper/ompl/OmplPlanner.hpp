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
#include <robot_model/RobotModel.hpp>
#include "RobotFreeJointParameter.hpp"
#include "OmplConfig.hpp"
#include "HandleOmplConfig.hpp"


namespace motion_planners
{
    
class OmplPlanner: public motion_planners::AbstractPlanner
{
    public:
        /**
        * @brief OMPL planner class constructor
        */
        OmplPlanner();
        /**
        * @brief OMPL class destructor
        */
        ~OmplPlanner();
        /**
        * @brief Initialize the motion planner
        * @param robot_model Robot model.
        * @param planner_specfic Configuration file for planner.
        * @return Returns true, when the initialization is successful or else returns false
        */
        bool initializePlanner(std::shared_ptr<robot_model::RobotModel>& robot_model, std::string config_file_path);
        /**
        * @brief Reinitialize the motion planner. This function is used only in STOMP planner.
        * @return Returns true, when the initialization is successful or else returns false
        */
        bool reInitializePlanner(){return false;}
        /**
        * @brief Reinitialize the time step in the motion planner. This function is used in STOMP planner.
        */
        bool reInitializeTimeSteps(const int &num_time_steps){return false;}
        /**
        * @brief  Solve the planning problem.
        * @param  solution Planner solution.
        * @param  planner_status Planner status.        
        * @return Returns true, when the planning is successful else returns false.
        */
        bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status);
        /**
        * @brief  Set the start and goal to the planner.
        * @param  start Start configuration in joint space. 
        * @param  goal Goal configuration in joint space.        
        */
        void setStartGoalTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal);
        /**
        * @brief  Set the contraints for the planner. This function is used in OMPL planner.
        * @param  constraints Constraints.        
        */
        void setConstraints(const ConstraintPlanning constraints){constraints_ = constraints;};
        /**
        * @brief  Update the initial trajectory. This function is used only for the STOMP planner.
        * @param  trajectory Initial trajectory.
        * @return Returns true, when the update is successful or else returns false.
        */
        bool updateInitialTrajectory(const base::JointsTrajectory &trajectory);
        /**
        * @brief  Get the initial trajectory. This function is used only for the STOMP planner.        
        * @return Initial trajectory.
        */
        base::JointsTrajectory getInitialTrajectory(){ return base::JointsTrajectory();}
        /**
        * @brief  Get the number of iteration used for the planning problem. This function is used only for the STOMP planner.        
        * @return Number of iteration.
        */
        size_t getNumOfIterationsUsed(){return 0;}        
    private:

        std::map <std::string, double> lower_limits_;
        std::map <std::string, double> upper_limits_;
        base::samples::Joints start_joint_values_, goal_joint_values_;
        base::samples::RigidBodyState start_pose_, goal_pose_;
        std::size_t planning_group_joints_size_;
        ConstraintPlanning constraints_;
        double collision_cost_;

        //planner_type_ type_of_planner_;
        ompl_config::OmplConfig ompl_config_;
        int number_of_dimensions_;
        double max_solve_time_;
        unsigned int maxSteps_smoothing_ ;

        std::map<std::string, std::string> planner_specific_parameters_kv_;

        std::shared_ptr<ompl::geometric::PathGeometric> solution_path_ptr_;

        std::vector <std::map<std::string , double > >  path_joint_values_;
        std::vector <std::map<std::string , double > >  cartesian_path_;
        std::vector <std::string> orientation_constraint_names_ , position_constraint_names_;
        kinematics_library::AbstractKinematicPtr kin_solver_;

        bool setUpPlanningTaskInJointSpace(PlannerStatus &planner_status);
        bool setUpPlanningTaskInCartesianSpace(PlannerStatus &planner_status);	


        double getConstrainDifference( const base::Vector3d &value, const base::Vector3d &tolerance, const base::Vector3d &current_value,
                                       const double &constraint_weight);
        
        bool jointSpaceStateValidChecker(const ompl::base::State *state);
        bool collisionCheckerStateValid(const ompl::base::State *state);
        void setUpPlanner (ompl::base::PlannerPtr &planner, ompl::base::SpaceInformationPtr &space_information, ompl::base::ProblemDefinitionPtr &problem_definition_ptr);
        bool solveProblem(ompl::base::PlannerPtr &planner,  const ompl::base::SpaceInformationPtr &space_information, const ompl::base::ProblemDefinitionPtr &problem_definition_ptr, 
                          base::JointsTrajectory &solution, PlannerStatus &planner_status);
        void setPlannerStatus(const ompl::base::PlannerStatus::StatusType &ompl_status, motion_planners::PlannerStatus &planner_status);
        bool checkStartState(const base::samples::Joints &current_robot_status, PlannerStatus &planner_status );
        bool solveTaskInCartesianSpace(base::JointsTrajectory &solution, PlannerStatus &planner_status);
        bool solveTaskInJointSpace(base::JointsTrajectory &solution, PlannerStatus &planner_status);
        bool cartesianSpaceStateValidityChecker(const ompl::base::State *state);  
        void simplifySolution(const ompl::base::ProblemDefinitionPtr &problem, ompl::geometric::PathSimplifier &path_simplifier, 
                              unsigned int step_size = 1, double duration = 0.0);
        bool setLimitsFromConstraints( const std::vector<std::string> &value_name, const ConstraintValues &constraint);

};
}// end namespace motion_planners
#endif // OMPLPLANNER_HPP
