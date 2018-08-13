#include <vector>
#include <string>

#include <MotionPlanners.hpp>

/** \file CollisionDetection.hpp
*    \brief Factory class for the AbstractCollisionDetection class.
*/

using namespace motion_planners;


MotionPlanners::MotionPlanners(PlannerConfig planner_config)
{
    planner_config_ = planner_config;
}

MotionPlanners::~MotionPlanners()
{}


bool MotionPlanners::initialize(PlannerStatus &planner_status)
{

    // CAUTION: Don't use different collision library for robot and world.
    // IN FCL wrapper the base pointer is downcasted.
    collision_detection::AbstractCollisionPtr robot_collision_detector = collision_factory_.getCollisionDetector(collision_detection::FCL);
    collision_detection::AbstractCollisionPtr world_collision_detector = collision_factory_.getCollisionDetector(collision_detection::FCL);
    kinematics_library::RobotKinematicsPtr robot_kinematics =  kinematics_factory_.getKinematicsSolver(planner_config_.kinematics_config);
    robot_kinematics->initialise(planner_status.kinematic_status);
    robot_model_.reset(new RobotModel(planner_config_.robot_model.urdf_file, planner_config_.robot_model.srdf_file, planner_config_.robot_model.planning_group_name));
    robot_model_->setRobotCollisionDetector(robot_collision_detector);
    robot_model_->setWorldCollisionDetector(world_collision_detector); 
    robot_model_->setKinematicsSolver(robot_kinematics);
    if(!robot_model_->initialization())
    {
	planner_status.statuscode = PlannerStatus::ROBOTMODEL_INITIALISATION_FAILED;
	return false;
    }
    
    PlannerFactory planner_factory;
    
    planner_ = planner_factory.getPlannerTask(planner_config_.planner);
    
    if(!planner_->initializePlanner(robot_model_, planner_config_.planner_specific_config))
    {
	planner_status.statuscode = PlannerStatus::PLANNER_INITIALISATION_FAILED;
	return false;
    }
    
    std::string base_link, tip_link;
    planning_group_joints_.clear();
    robot_model_->getPlanningGroupJointinformation(planner_config_.robot_model.planning_group_name, planning_group_joints_, base_link, tip_link);
    
    goal_pose_.position = Eigen::Vector3d::Zero();
    goal_pose_.orientation = Eigen::Quaterniond::Identity();
    
    return true;
}

bool MotionPlanners::checkStartState(const base::samples::Joints &current_robot_status, PlannerStatus &planner_status )
{
    // check whether the start state is in collision
    
    robot_model_->updateJointGroup(current_robot_status);
            
    if(!robot_model_->isStateValid())
    {
        planner_status.statuscode = PlannerStatus::START_STATE_IN_COLLISION;
	collision_object_names_ = robot_model_->getCollisionObjectNames();
	return false;
    }
    else
    {
	// assign the start joint values from current robot status	  
	initial_joint_status_.clear();
	initial_joint_status_.resize(planning_group_joints_.size());
	
        for(size_t i = 0; i < planning_group_joints_.size(); i++)
        {
	    try
	    {
		base::JointState current_jointstate = current_robot_status.getElementByName(planning_group_joints_.at(i).first);		
		
		initial_joint_status_.names.at(i) = planning_group_joints_.at(i).first;
		initial_joint_status_.elements.at(i) = current_jointstate;		
	    }
	    catch(base::samples::Joints::InvalidName e) //Only catch exception to write more explicit error msgs
	    {
 		LOG_ERROR("[MotionPlanners]: Joint %s is given in planning group but is not available in the given start value", planning_group_joints_.at(i).first.c_str());
		return false;
	    }
        }
    }
    
    return true;
}

bool MotionPlanners::checkGoalState(const base::samples::Joints &goal, PlannerStatus &planner_status )
{
    // check whether the goal state is in collision        
    robot_model_->updateJointGroup(goal);
    if(!robot_model_->isStateValid())
    {
	planner_status.statuscode = PlannerStatus::GOAL_STATE_IN_COLLISION;	    
	collision_object_names_ = robot_model_->getCollisionObjectNames();
    }
    else
    {
	planner_status.statuscode = PlannerStatus::PLANNING_REQUEST_SUCCESS;
	return true;
    }
    
    return false;    
}

bool MotionPlanners::assignPlanningRequest(const base::samples::Joints &start_jointvalues, const base::samples::Joints &target_jointvalues,
					      std::string &planningGroupName, PlannerStatus &planner_status)
{
    
    if (checkStartState(start_jointvalues, planner_status))
    {
	// assign the goal joint values from the target joint status
	goal_joint_status_.clear();
	goal_joint_status_.resize(planning_group_joints_.size());
	for(size_t i = 0; i < planning_group_joints_.size(); i++)
	{
	    try
	    {
		goal_joint_status_.names.at(i)		= planning_group_joints_.at(i).first;
		goal_joint_status_.elements.at(i)	= target_jointvalues.getElementByName(planning_group_joints_.at(i).first);
	    }
	    catch(base::samples::Joints::InvalidName e) //Only catch exception to write more explicit error msgs
	    {
 		LOG_ERROR("[MotionPlanners]: Joint %s is given in planning group but is not available in the given target value", planning_group_joints_.at(i).first.c_str());
		return false;
	    }
	}
	
	if(checkGoalState(goal_joint_status_, planner_status))
	    return true;
	else
	    return false;	
        
    }
    
    return false;

}


bool MotionPlanners::assignPlanningRequest(const base::samples::Joints &start_jointvalues, const base::samples::RigidBodyState &target_pose,
					      std::string &planningGroupName, PlannerStatus &planner_status)
{
    
    if (checkStartState(start_jointvalues, planner_status))
    {	 

	// assign the goal joint values from the target joint status
	goal_pose_ = target_pose;	
	// assign the goal joint values from the target joint status
	goal_joint_status_.clear();
	goal_joint_status_.resize(planning_group_joints_.size());
	
	robot_model_->robot_kinematics_->solveIK(goal_pose_.position, goal_pose_.orientation, start_jointvalues,
						 ik_solution_, planner_status.kinematic_status);
	std::cout<<"aff = "<<planner_status.kinematic_status.statuscode<<std::endl;
	if(planner_status.kinematic_status.statuscode == kinematics_library::KinematicsStatus::IK_FOUND)
	{std::cout<<"Found IK "<<std::endl;
	    for(size_t i = 0; i < planning_group_joints_.size(); i++)
	    {
		try
		{
		    
		    goal_joint_status_.names.at(i)	= planning_group_joints_.at(i).first;
		    goal_joint_status_.elements.at(i)	= ik_solution_.getElementByName(planning_group_joints_.at(i).first);
		}
		catch(base::samples::Joints::InvalidName e) //Only catch exception to write more explicit error msgs
		{
		    LOG_ERROR("[MotionPlanners]: Joint %s is given in planning group but is not available for the goal value", planning_group_joints_.at(i).first.c_str());
		    return false;
		}
	    }
	    
	    if(checkGoalState(goal_joint_status_, planner_status))
		return true;
	    else
		return false;	    
	}
	std::cout<<"beforedd = "<<planner_status.kinematic_status.statuscode<<std::endl;
    }
    
    return false;

}

bool MotionPlanners::solve(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{    
    planner_->updateInitialTrajectory(initial_joint_status_, goal_joint_status_, planner_status);
    
    std::cout<<"SSSSSSSSTART "<<std::endl;
    for(size_t i = 0; i < planning_group_joints_.size(); i++)
	std::cout<<initial_joint_status_.elements.at(i).position<<std::endl;
    std::cout<<"----------------"<<std::endl;
    std::cout<<"EEEEEnd "<<std::endl;
    for(size_t i = 0; i < planning_group_joints_.size(); i++)
	std::cout<<goal_joint_status_.elements.at(i).position<<std::endl;
    std::cout<<"----------------"<<std::endl;
    std::cout<<"beforei solve = "<<planner_status.statuscode<<std::endl;
    planner_->solve(solution, planner_status);
    std::cout<<"after solve = "<<planner_status.statuscode<<std::endl;
    
    return true;
    
    
}