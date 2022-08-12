
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
        StompPlanner();

        ~StompPlanner();

        bool initializePlanner(std::shared_ptr<robot_model::RobotModel>& robot_model, std::string config_file_path);

        bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status);

        void setStartGoalTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal);

        bool updateInitialTrajectory(const base::JointsTrajectory &trajectory);

        bool reInitializePlanner(const int &num_time_steps);
        
        base::JointsTrajectory getInitialTrajectory();

        void setConstraints(const ConstraintPlanning constraints){constraints_ = constraints;}

        double getMovementDeltaTime();
        
        size_t getNumOfIterationsUsed() { return num_iterations_;}

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
