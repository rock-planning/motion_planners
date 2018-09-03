#ifndef TRAJECTORYOPTIMIZATION_HPP_
#define TRAJECTORYOPTIMIZATION_HPP_

#include <vector>
#include <string>
#include <base/samples/Joints.hpp>
#include "Config.hpp"
#include <base/JointsTrajectory.hpp>
#include <base/samples/Pointcloud.hpp>
#include <collision_detection/CollisionDetection.hpp>
#include <kinematics_library/KinematicsFactory.hpp>
#include "RobotModel.hpp"
#include "PlannerFactory.hpp"
#include "abstract/AbstractPlanner.hpp"



/** \file CollisionDetection.hpp
*    \brief Factory class for the AbstractCollisionDetection class.
*/

namespace motion_planners
{

/**
 * @class MotionPlanners
 * @brief MotionPlanners class.
 */
class MotionPlanners
{

  public:
    /**
    * @brief  constructor
    */
    MotionPlanners(Config config);
    /**
    * @brief  destructor
    */
    ~MotionPlanners();
    
    bool initialize(PlannerStatus &error_status);

    bool assignPlanningRequest(const base::samples::Joints &start_jointvalues, const base::samples::Joints &target_jointvalues,
			      std::string &planningGroupName, PlannerStatus &planner_status);
    
    bool assignPlanningRequest(const base::samples::Joints &start_jointvalues, const base::samples::RigidBodyState &target_pose,
			       std::string &planningGroupName, PlannerStatus &planner_status);
    
    void updatePointcloud(const base::samples::Pointcloud &pt_cloud, const Eigen::Vector3d &sensor_origin);    
    
    bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status);
    
    void getEnviornmentPointcloud(base::samples::Pointcloud &env_ptcloud);
    
    std::vector< std::pair<std::string, std::string> > getCollisionObjectNames()
    {
	return collision_object_names_;
    }
    
    base::JointsTrajectory planner_solution_;
    
    AbstractPlannerPtr planner_;
    
  private:     

    
    std::shared_ptr<RobotModel> robot_model_;
    
    collision_detection::CollisionDetection collision_factory_;
    
    std::vector< std::pair<std::string, std::string> > collision_object_names_;
    
    kinematics_library::KinematicsFactory kinematics_factory_;
    
    Config config_;
    
    std::vector< std::pair<std::string,urdf::Joint> > planning_group_joints_;
    
    base::samples::Joints initial_joint_status_, goal_joint_status_;
    
    base::samples::RigidBodyState goal_pose_;
    
    base::commands::Joints ik_solution_;
    
    pcl::PointCloud<pcl::PointXYZ> env_pcl_cloud_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr self_filtered_env_cloud_;
    
    bool checkStartState(const base::samples::Joints &current_robot_status, PlannerStatus &planner_status);
    
    bool checkGoalState(const base::samples::Joints &goal, PlannerStatus &planner_status);

};

};

#endif

