
#ifndef STOMPPLANNER_HPP_
#define STOMPPLANNER_HPP_

#include <sys/stat.h> //needed to create director (mkdir function)

#include <abstract/AbstractPlanner.hpp>
#include "OptimizationTask.hpp"
#include "HandleStompConfig.hpp"


namespace motion_planners
{

class StompPlanner: public motion_planners::AbstractPlanner
{
    public:
        /**
        * @brief STOMP planner class constructor
        */
        StompPlanner();
        /**
        * @brief STOMP class destructor
        */
        ~StompPlanner();
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
        bool reInitializePlanner();
        /**
        * @brief Reinitialize the time step in the motion planner. This function is used in STOMP planner.
        */
        bool reInitializeTimeSteps(const int &num_time_steps);
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
        void setConstraints(const ConstraintPlanning constraints){constraints_ = constraints;}
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
        base::JointsTrajectory getInitialTrajectory();
        /**
        * @brief  Get the number of iteration used for the planning problem. This function is used only for the STOMP planner.        
        * @return Number of iteration.
        */
        size_t getNumOfIterationsUsed() { return num_iterations_;}
        
        double getMovementDeltaTime();
        
        

    private:
        boost::shared_ptr<stomp::Stomp> stomp_;
        stomp::StompConfig stomp_config_;
        stomp::DebugConfig debug_config_;
        std::shared_ptr<OptimizationTask> optimization_task_;
        stomp::CovariantMovementPrimitive tmp_policy;        
        ConstraintPlanning constraints_;
        size_t num_iterations_;
};
}// end planner

#endif
