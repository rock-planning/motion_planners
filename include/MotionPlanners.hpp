#ifndef MOTIONPLANNERS_HPP_
#define MOTIONPLANNERS_HPP_

#include <vector>
#include <string>
#include <base/samples/Joints.hpp>
#include <base/JointsTrajectory.hpp>
#include <collision_detection/CollisionFactory.hpp>
#include <kinematics_library/KinematicsFactory.hpp>
#include <robot_model/RobotModel.hpp>
#include "PlannerFactory.hpp"
#include "abstract/AbstractPlanner.hpp"
#include "Config.hpp"


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
                                   PlannerStatus &planner_status);

        bool assignPlanningRequest( const base::samples::Joints &start_jointvalues, const base::samples::RigidBodyState &target_pose,
                                    PlannerStatus &planner_status);
        
        bool assignPlanningRequest( const base::samples::Joints &start_jointvalues, const ConstraintPlanning &constrainted_target,
                            PlannerStatus &planner_status);

        bool assignPlanningRequest( const base::samples::Joints &start_jointvalues, const std::string &target_group_state,
                                    PlannerStatus &planner_status);

        const base::samples::Joints &getGoalJointAngles();

        bool usePredictedTrajectory( base::JointsTrajectory &solution, PlannerStatus &planner_status);

        void setStartAndGoal();

        void assignOctomapPlanningScene(const std::shared_ptr<octomap::OcTree> &octomap);

        void updateOctomap(const std::shared_ptr<octomap::OcTree> &octomap);

        bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status, double &time_taken);

        bool handleCollisionObjectInWorld(const motion_planners::ModelObject &known_object);

        bool handleGraspObject(const motion_planners::ModelObject &known_object);

        std::vector< std::pair<std::string, std::string> > getCollisionObjectNames()
        {
            return collision_object_names_;
        }

        base::JointsTrajectory planner_solution_;

        AbstractPlannerPtr planner_;

    private:

        std::shared_ptr<RobotModel> robot_model_;

        collision_detection::CollisionFactory collision_factory_;

        std::vector< std::pair<std::string, std::string> > collision_object_names_;

        kinematics_library::KinematicsFactory kinematics_factory_;

        Config config_;

        std::vector< std::pair<std::string,urdf::Joint> > planning_group_joints_;

        std::map< std::string, std::map< std::string,double > > named_group_states_;

        base::samples::Joints initial_joint_status_, goal_joint_status_;

        base::samples::RigidBodyState goal_pose_;

        std::vector<base::commands::Joints> ik_solution_;
        
        ConstraintPlanning constrainted_target_;

        bool checkStartState(const base::samples::Joints &current_robot_status, PlannerStatus &planner_status);

        bool checkGoalState(const base::samples::Joints &goal, PlannerStatus &planner_status);

        bool convertModelObjectToURDFCollision(const motion_planners::ModelObject &known_object, std::shared_ptr<urdf::Collision> collision_object);

        void createNamedGroupStates(boost::shared_ptr<srdf::Model> srdf_model);


};

};

#endif

