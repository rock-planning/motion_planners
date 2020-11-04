#ifndef DUALARMMOTIONPLANNERS_HPP_
#define DUALARMMOTIONPLANNERS_HPP_

#include "motion_planners/MotionPlanners.hpp"

namespace motion_planners
{

class DualArmMotionPlanners: public MotionPlanners
{
public:
    DualArmMotionPlanners(Config motion_planner_config, KLCConfig klc_config);
    ~DualArmMotionPlanners();
    bool initializeDualArmConfig(PlannerStatus &planner_status);

    bool assignDualArmPlanningRequest( const base::samples::Joints &start_jointvalues, 
                                       const base::samples::Joints &target_jointvalues, PlannerStatus &planner_status);

private:
    kinematics_library::AbstractKinematicPtr active_chain_kin_solver_, passive_chain_kin_solver_;
    KLCConfig klc_config_;
};
}

#endif