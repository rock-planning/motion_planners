#ifndef MOTIONPLANNERS_HPP_
#define MOTIONPLANNERS_HPP_

#include <vector>
#include <string>
#include <math.h>
#include <base/samples/Joints.hpp>
#include <base/JointsTrajectory.hpp>
#include <collision_detection/CollisionFactory.hpp>
#include <kinematics_library/KinematicsFactory.hpp>
#include <robot_model/RobotModel.hpp>
#include "PlannerFactory.hpp"
#include "abstract/AbstractPlanner.hpp"
#include "motion_planners/Config.hpp"


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
        * @brief Constructor.
        * @param config Configuration for motion planner.
        */
        MotionPlanners(Config config);
        /**
        * @brief Destructor
        */
        ~MotionPlanners();
        /**
        * @brief Initialize the motion planner.
        * @param error_status The error status tells why the initialization failed.
        * @return Returns true, when the initialization is successful or else returns false.
        */
        bool initialize(PlannerStatus &error_status);
        /**
        * @brief Reinitialize the motion planner. This function is used in STOMP planner.
        * @return Returns true, when the initialization is successful or else returns false.
        */
        bool reInitializePlanner();
        /**
        * @brief  Assign a planning request in joint space
        * @param  start_jointvalues Joints value at the start configuration
        * @param  target_jointvalues Joints value at the target configuration
        * @param  planner_status Planner status output the status of the request.
        * @return Returns true, when the planning request is successful or else returns false
        */
        bool assignPlanningRequest(const base::samples::Joints &start_jointvalues, const base::samples::Joints &target_jointvalues,
                                   PlannerStatus &planner_status);
        /**
        * @brief  Assign a planning request in Cartesian space
        * @param  start_jointvalues Joints value at the start configuration
        * @param  target_pose Pose at the target configuration
        * @param  planner_status Planner status output the status of the request.
        * @return Returns true, when the planning request is successful else returns false
        */
        bool assignPlanningRequest( const base::samples::Joints &start_jointvalues, const base::samples::RigidBodyState &target_pose,
                                    PlannerStatus &planner_status);
        /**
        * @brief  Assign a planning request in constraint joint space.
        * @param  start_jointvalues Joints value at the start configuration.
        * @param  ConstraintPlanning Constraints for the planner. Only sampling-based planners (OMPL) uses this feature.
        * @param  planner_status Planner status output the status of the request.
        * @return Returns true, when the planning request is successful else returns false. 
        */
        bool assignPlanningRequest( const base::samples::Joints &start_jointvalues, const ConstraintPlanning &constrainted_target,
                                    PlannerStatus &planner_status);
        /**
        * @brief  Assign a planning request in joint space.
        * @param  start_jointvalues Joints value at the start configuration.
        * @param  target_group_state Get the target joints value from the SRDF.
        * @param  planner_status Planner status output the status of the request.
        * @return Returns true, when the planning request is successful else returns false.
        */
        bool assignPlanningRequest( const base::samples::Joints &start_jointvalues, const std::string &target_group_state,
                                    PlannerStatus &planner_status);
        /**
        * @brief  Get start joint angles.
        * @return Returns start joint angles.
        */
        inline const base::samples::Joints &getStartJointAngles(){return initial_joint_status_;}
        /**
        * @brief  Get goal joint angles.
        * @return Returns goal joint angles.
        */
        inline const base::samples::Joints &getGoalJointAngles(){return goal_joint_status_;}
        /**
        * @brief  Assign an initial trajectory to warm start an optimization-based planner.
        * @param  solution Initial trajectory.
        * @param  planner_status Planner status output the status of the request.
        * @return Returns true, when the assignment is successful else returns false.
        */
        bool usePredictedTrajectory( base::JointsTrajectory &solution, PlannerStatus &planner_status);
        /**
        * @brief  Set the start and goal to the planner. This function is called after assigning the planning request.
        */
        void setStartAndGoal();
        /**
        * @brief  Assign an octomap for the planning scene.
        * @param  octomap Octomap representing the planning scene.
        */
        void assignOctomapPlanningScene(const std::shared_ptr<octomap::OcTree> &octomap);
        /**
        * @brief  Update an octomap for the planning scene.
        * @param  octomap Octomap representing the planning scene.
        */
        void updateOctomap(const std::shared_ptr<octomap::OcTree> &octomap);
        /**
        * @brief  Solve the planning problem.
        * @param  solution Planner solution.
        * @param  planner_status Planner status.
        * @param  time_taken Time taken to plan in seconds.
        * @return Returns true, when the planning is successful else returns false.
        */
        bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status, double &time_taken);
        /**
        * @brief  Handle collision object in the planning scene. Collision object can be added or removed.
        * @param  known_object collision object.
        * @return Returns true, when the request is successful else returns false.
        */
        bool handleCollisionObjectInWorld(const motion_planners::ModelObject &known_object);
        /**
        * @brief  Handle grasped object.
        * @param  known_object Object to be added or removed from the gripper.
        * @return Returns true, when the request is successful else returns false.
        */
        bool handleGraspObject(const motion_planners::ModelObject &known_object);
        /**
        * @brief  Get robot model
        * @return robot model.
        */
        RobotModelPtr getRobotModel(){return robot_model_;}
        /**
        * @brief  Get collision object names.
        * @return Collision object names.
        */
        collision_detection::CollisionLinksName getCollidedObjectsNames();
        
        base::JointsTrajectory planner_solution_;

        AbstractPlannerPtr planner_;

    protected:
        /**
        * @brief  Check the start state for the planning request.
        * @param  current_robot_status Collision checking is performed for this joints value.
        * @param  planner_status Planner status output the status of the request.
        * @return Returns true, when the s is successful else returns false.
        */
        bool checkStartState(const base::samples::Joints &current_robot_status, PlannerStatus &planner_status);
        /**
        * @brief  Check the start state for the planning request.
        * @param  goal Collision checking is performed for this joints value.
        * @param  planner_status Planner status output the status of the request.
        * @return Returns true, when the request is successful else returns false.
        */
        bool checkGoalState(const base::samples::Joints &goal, PlannerStatus &planner_status);
        /**
        * @brief  Convert the model object to URDF type collision object.
        * @param  known_object Object to be converted.
        * @param  collision_object URDF based object.
        * @return Returns true, when the conversion is successful else returns false.
        */
        bool convertModelObjectToURDFCollision(const motion_planners::ModelObject &known_object, std::shared_ptr<urdf::Collision> collision_object);
        /**
        * @brief  Created a named group state from the SRDF file and store in the variable named_group_states_.
        * @param  srdf_model SRDF file.
        */
        void createNamedGroupStates(boost::shared_ptr<srdf::Model> srdf_model);
        /**
        * @brief  Get collision objects that should be disabled.
        * @param  disabled_collision_pair Collision pair names that need to be disabled.
        * @return Returns collision pairs that are extracted from the URDF based on the given collision object names.
        */
        std::vector <std::pair<std::string,std::string> > assignDisableCollisionObject(const collision_detection::CollisionLinksName &disabled_collision_pair);
        /**
        * @brief  Check NAN in the given joint value
        * @return Returns true if there is no NAN, else false.
        */
        bool checkNaN(base::samples::Joints joint_value);
        /**
        * @brief  Assign kinematic solver to the robot model
        * @param  kinematics_config Kinematics config
        * @param  robot_kinematics The kinematics solver pointer.
        * @param  planner_status Planner status output the status of the request.
        * @return Returns true, when the assignment is successful else returns false.
        */
        bool assignKinematicsToRobotModel(const kinematics_library::KinematicsConfig &kinematics_config, 
                                          kinematics_library::AbstractKinematicPtr &robot_kinematics,
                                          PlannerStatus &planner_status);


        std::shared_ptr<RobotModel> robot_model_;

        collision_detection::CollisionFactory collision_factory_;

        std::vector< std::pair<std::string, std::string> > collision_object_names_;

        kinematics_library::KinematicsFactory kinematics_factory_;

        std::vector< std::pair<std::string,urdf::Joint> > planning_group_joints_;

        std::map< std::string, std::map< std::string,double > > named_group_states_;

        base::samples::Joints initial_joint_status_, goal_joint_status_;

        base::samples::RigidBodyState goal_pose_;

        std::vector<base::commands::Joints> ik_solution_;
        
        ConstraintPlanning constrainted_target_;

        
    private:
        Config config_;

        kinematics_library::AbstractKinematicPtr kin_solver_;
};

};

#endif

