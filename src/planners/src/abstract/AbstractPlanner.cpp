#include "abstract/AbstractPlanner.hpp"

namespace motion_planners
{

AbstractPlanner::AbstractPlanner()
{
    root_name_ = "";base_name_ = "";tip_name_ = "";
}

bool AbstractPlanner::assignPlanningJointInformation(std::shared_ptr<robot_model::RobotModel> robot_model)
{
    robot_model_ = robot_model;
    
    // get the different frame name
    root_name_ = robot_model_->getWorldFrameName();
    base_name_ = robot_model_->getBaseFrameName();
    tip_name_  = robot_model_->getTipFrameName();

    planning_group_name_ = robot_model_->getPlanningGroupName();

    //get planning group joint names.
    robot_model_->getPlanningGroupJointInformation(planning_group_name_, planning_group_joints_, planning_group_joints_name_);
}

}
