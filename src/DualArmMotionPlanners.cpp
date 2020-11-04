#include <vector>
#include <string>

#include <motion_planners/DualArmMotionPlanners.hpp>


using namespace motion_planners;


DualArmMotionPlanners::DualArmMotionPlanners(Config motion_planner_config, KLCConfig klc_config):MotionPlanners(motion_planner_config)
{
    klc_config_ = klc_config;
}

DualArmMotionPlanners::~DualArmMotionPlanners()
{}

bool DualArmMotionPlanners::initializeDualArmConfig(PlannerStatus &planner_status)
{
    // first initialise for the active arm
    if(!initialize(planner_status))
        return false;

    // now assign the kinematics solver of the passive arm to the robot model
    if(!assignKinematicsToRobotModel(klc_config_.passive_chain_kin_config, passive_chain_kin_solver_, planner_status))
        return false;   

    return true;
}

bool DualArmMotionPlanners::assignDualArmPlanningRequest( const base::samples::Joints &start_jointvalues, 
                                                          const base::samples::Joints &target_jointvalues, PlannerStatus &planner_status)
{
    // check collision checking for the active arm
    if(!assignPlanningRequest( start_jointvalues, target_jointvalues, planner_status) )        
        return false;

    constrainted_target_.use_constraint = motion_planners::KLC_CONSTRAINT;
    constrainted_target_.klc_constraint = klc_config_.klc_constraint;

    return true;
}