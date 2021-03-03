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
        OmplPlanner();
        ~OmplPlanner();
        bool initializePlanner(std::shared_ptr<robot_model::RobotModel>& robot_model, std::string config_file_path);
        bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status);		
        void updateInitialTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal, PlannerStatus &planner_status);
        void setStartGoalTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal);
        void setConstraints(const ConstraintPlanning constraints){constraints_ = constraints;};
        bool updateInitialTrajectory(const base::JointsTrajectory &trajectory);
        base::JointsTrajectory getInitialTrajectory(){ return base::JointsTrajectory();}
        size_t getNumOfIterationsUsed(){return 0;}        
    private:

        std::map <std::string, double> lower_limits_;
        std::map <std::string, double> upper_limits_;
        base::samples::Joints start_joint_values_, goal_joint_values_;
        base::samples::RigidBodyState start_pose_, goal_pose_;
        std::size_t planning_group_joints_size_;
        // kinematics loop closure
        base::samples::RigidBodyState active_chain_pose_, passive_chain_pose_;
        Eigen::Affine3d klc_offset_pose_, passive_active_offset_;        
        kinematics_library::AbstractKinematicPtr active_chain_kin_solver_, passive_chain_kin_solver_;
        std::vector<base::commands::Joints> passive_chain_projected_state_;
        base::JointsTrajectory passive_chain_solution_;
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
        bool kinematicLoopClosureValidChecker(const ompl::base::State *state);  
//         void setCartesianConstraints(CartesianContraints constraints);
        void simplifySolution(const ompl::base::ProblemDefinitionPtr &problem, ompl::geometric::PathSimplifier &path_simplifier, 
                              unsigned int step_size = 1, double duration = 0.0);
        bool setLimitsFromConstraints( const std::vector<std::string> &value_name, const ConstraintValues &constraint);

        bool kinematicLoopClosureProjection(const base::samples::Joints &joint_values, 
                                            std::vector<base::commands::Joints> &projected_state);

        bool calculateKLCOffset();

        bool checkPassiveChainCollision(PlannerStatus &planner_status);

        bool calculatePassiveChainIKSoln(const base::samples::Joints &active_chain_joints, const base::samples::Joints &passive_chain_joints, 
                                         std::vector<base::commands::Joints> &passive_chain_iksoln, PlannerStatus &planner_status);
        
        void convertOmplStateToBaseJoints(const ompl::base::State *state, base::samples::Joints &joint_values);
};
}// end namespace motion_planners
#endif // OMPLPLANNER_HPP
