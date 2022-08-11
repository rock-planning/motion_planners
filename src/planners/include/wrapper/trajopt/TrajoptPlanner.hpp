
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
        TrajoptPlanner();

        ~TrajoptPlanner();

        bool initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string config_file_path);

        bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status);

        void setStartGoalTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal);

        void setConstraints(const ConstraintPlanning constraints){};

        bool updateInitialTrajectory(const base::JointsTrajectory &trajectory);

        bool reInitializePlanner(std::map<std::string, int> parameters){return false;}
        
        base::JointsTrajectory getInitialTrajectory(){ return base::JointsTrajectory();}
        
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
