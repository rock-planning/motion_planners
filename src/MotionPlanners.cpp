#include <vector>
#include <string>

#include <MotionPlanners.hpp>

/** \file CollisionDetection.hpp
*    \brief Factory class for the AbstractCollisionDetection class.
*/

using namespace motion_planners;


MotionPlanners::MotionPlanners(Config config)
{
    config_ = config;
}

MotionPlanners::~MotionPlanners()
{}


bool MotionPlanners::initialize(PlannerStatus &planner_status)
{

    // CAUTION: Don't use different collision library for robot and world.
    // IN FCL wrapper the base pointer is downcasted.
    collision_detection::AbstractCollisionPtr robot_collision_detector = collision_factory_.getCollisionDetector(collision_detection::FCL);
    collision_detection::AbstractCollisionPtr world_collision_detector = collision_factory_.getCollisionDetector(collision_detection::FCL);
    kinematics_library::RobotKinematicsPtr robot_kinematics =  kinematics_factory_.getKinematicsSolver(config_.planner_config.kinematics_config);
    robot_kinematics->initialise(planner_status.kinematic_status);
    robot_model_.reset(new RobotModel(config_.planner_config.robot_model.urdf_file, config_.planner_config.robot_model.srdf_file, 
				      config_.planner_config.robot_model.planning_group_name));
    robot_model_->setRobotCollisionDetector(robot_collision_detector);
    robot_model_->setWorldCollisionDetector(world_collision_detector); 
    robot_model_->setKinematicsSolver(robot_kinematics);
    if(!robot_model_->initialization())
    {
	planner_status.statuscode = PlannerStatus::ROBOTMODEL_INITIALISATION_FAILED;
	return false;
    }
    
    PlannerFactory planner_factory;
    
    planner_ = planner_factory.getPlannerTask(config_.planner_config.planner);
    
    if(!planner_->initializePlanner(robot_model_, config_.planner_config.planner_specific_config))
    {
	planner_status.statuscode = PlannerStatus::PLANNER_INITIALISATION_FAILED;
	return false;
    }
    
    std::string base_link, tip_link;
    planning_group_joints_.clear();
    robot_model_->getPlanningGroupJointinformation(config_.planner_config.robot_model.planning_group_name, planning_group_joints_, base_link, tip_link);
    
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

void MotionPlanners::getEnviornmentPointcloud(base::samples::Pointcloud &env_ptcloud)
{
    env_ptcloud.points.clear();
    
    env_ptcloud.points.resize(self_filtered_env_cloud_->size());
    
    for (size_t i = 0;  i < self_filtered_env_cloud_->size() ; ++i)
    {	
	env_ptcloud.points[i].x() = self_filtered_env_cloud_->points[i].x;
	env_ptcloud.points[i].y() = self_filtered_env_cloud_->points[i].y;
	env_ptcloud.points[i].z() = self_filtered_env_cloud_->points[i].z;
    }

    env_ptcloud.time = base::Time::now();
}

void MotionPlanners::updatePointcloud(const base::samples::Pointcloud &pt_cloud, const Eigen::Vector3d &sensor_origin)
{
    //convert pointcloud to PCL-Pointcloud
    env_pcl_cloud_.clear();
    for (std::vector<base::Vector3d>::const_iterator it = pt_cloud.points.begin() ; it != pt_cloud.points.end(); ++it)
    {
	pcl::PointXYZ point((*it)[0],(*it)[1],(*it)[2]);
	env_pcl_cloud_.push_back(point);
    } 
    
    //self filtering  
    self_filtered_env_cloud_->clear();
    
    robot_model_->selfFilter(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(env_pcl_cloud_), 
					   self_filtered_env_cloud_, config_.env_config.env_frame, motion_planners::COLLISION);

    // add pointloud to collision model    
    robot_model_->updatePointcloud(self_filtered_env_cloud_, sensor_origin, config_.env_config.env_frame, 
				   config_.env_config.octree_resolution, config_.env_config.env_object_name);   
    
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
	
	robot_model_->robot_kinematics_->solveIK(goal_pose_, start_jointvalues,
						 ik_solution_, planner_status.kinematic_status);
	
	if(planner_status.kinematic_status.statuscode == kinematics_library::KinematicsStatus::IK_FOUND)
	{
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
    }    
    return false;
}

bool MotionPlanners::solve(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{    
    planner_->updateInitialTrajectory(initial_joint_status_, goal_joint_status_, planner_status);        
    
    planner_->solve(solution, planner_status);    
    
    return true;    
    
}