
#ifndef TRAJOPTPLANNER_HPP_
#define TRAJOPTPLANNER_HPP_

#include <abstract/AbstractPlanner.hpp>
#include <robot_model/RobotModel.hpp>
#include "json/json.h"
#include "trajopt/problem_description.hpp"
#include "sco/optimizers.hpp"
#include "utils/clock.hpp"
#include <wrapper/trajopt/RobotModelWrapper.h>
#include <wrapper/trajopt/FCLCollisionChecker.h>

using namespace trajopt;
using namespace Json;

namespace motion_planners
{

class TrajoptPlanner: public motion_planners::AbstractPlanner
{
    public:
        /**
        * @brief TrajOpt planner class constructor
        */
        TrajoptPlanner();
        /**
        * @brief TrajOpt class destructor
        */
        ~TrajoptPlanner();
        /**
        * @brief Initialize the motion planner
        * @param robot_model Robot model.
        * @param planner_specfic Configuration file for planner.
        * @return Returns true, when the initialization is successful or else returns false
        */
        bool initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string config_file_path);
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
        void setConstraints(const ConstraintPlanning constraints){};
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

        TrajOptProbPtr m_prb;
        BasicTrustRegionSQP m_opt;

        RobotModelWrapperPtr m_robot_model_wrapper;
        FCLCollisionCheckerPtr m_collision_checker_wrapper;

        YAML::Node m_input_config;

        TrajOptResultPtr m_results;

};
}// end planner

#endif
