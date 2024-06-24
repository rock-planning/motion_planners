#ifndef ABSTRACTPLANNERCONFIG_HPP_
#define ABSTRACTPLANNERCONFIG_HPP_

#include <string>
#include <vector>
#include <base/samples/Joints.hpp>
#include <kinematics_library/KinematicsConfig.hpp>

namespace motion_planners
{

enum Constraint
{
    POSITION_CONSTRAINT, ORIENTATION_CONSTRAINT, POSE_CONSTRAINT, 
    JOINTS_CONSTRAINT, NO_CONSTRAINT
};

struct ConstraintValues
{
    base::VectorXd value;
    // tolerance for roll, pitch and yaw. Please provide only positive value
    base::VectorXd tolerance;
};

struct ConstraintPlanning
{
    ConstraintPlanning():use_constraint(motion_planners::NO_CONSTRAINT){}
    
    Constraint use_constraint;
    // roll, pitch and yaw
    ConstraintValues orientation_constraint;
    // position constraint
    ConstraintValues position_constraint;
    // joint constraint
    ConstraintValues joint_constraint;
    // goal pose
    base::samples::RigidBodyState target_pose;
    // goal in joint space
    base::samples::Joints target_joints_value;
    
};

struct PlannerStatus
{
    enum StatusCode
    {
        // Planner found a path
        PATH_FOUND,
        // No path found
        NO_PATH_FOUND,
        // Start state is in collision
        START_STATE_IN_COLLISION,
        // Goal state is in collision
        GOAL_STATE_IN_COLLISION,
        // Start joint angle is not available
        START_JOINTANGLES_NOT_AVAILABLE,
        // Goal joint angle is not available
        GOAL_JOINTANGLES_NOT_AVAILABLE,
        // The constraint's upper and lower bounds are not within bounds
        // OMPL expect lower bounds lesser than the upper bounds
        CONSTRAINED_POSE_NOT_WITHIN_BOUNDS,        
        // Planning reuest is successfully created.
        PLANNING_REQUEST_SUCCESS,
        // The planner timeout
        TIMEOUT,
        // Invalid start state or no start state specified
        INVALID_START_STATE,
        // Invalid goal state
        INVALID_GOAL_STATE,
        // The goal is of a type that a planner does not recognize
        UNRECOGNIZED_GOAL_TYPE,
        // The planner found an approximate solution
        APPROXIMATE_SOLUTION,
        // The planner found an exact solution
        EXACT_SOLUTION,
        // Robot model initialisation failed
        ROBOTMODEL_INITIALISATION_FAILED,
        // Planner initialisation failed
        PLANNER_INITIALISATION_FAILED,
        // Constraint not given
        NO_CONSTRAINT_AVAILABLE,
        // Joint constraints value or tolerance size 
        // doesn't match with planning dimension size
        JOINT_CONSTRAINT_SIZE_ERROR,
        // The planner crashed
        CRASH,
        // Kinematics related error
        KINEMATIC_ERROR,
        /// invalid state
        INVALID
    }statuscode;

    kinematics_library::KinematicsStatus kinematic_status;

};

}// end motion_planners
#endif // ABSTRACTPLANNERCONFIG_HPP

