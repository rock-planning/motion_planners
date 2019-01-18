#ifndef OMPLCONFIG_HPP
#define OMPLCONFIG_HPP

#include <string>
#include <vector>


namespace ompl_config
{

enum PlannerType
{
    SBL, EST, LBKPIECE, BKPIECE, KPIECE, RRT, RRTConnect, RRTstar, PRM, PRMstar
};
//planner_type_;

/**
 *  @struct MotionPlanningParameters.
 * @brief This struct contains parameters used in the motion planning.
 */
        
struct OmplConfig
{
    OmplConfig():
        type_of_planner(ompl_config::RRTConnect), max_solve_time(1.0), max_step_smoothing(2),
        max_time_soln_simpilification(0.0), planner_specific_parameters_range("0.05") {}
    // planner types
    PlannerType type_of_planner;
    // maximum planning time
    double max_solve_time;
    // maximum step smoothing used in B-Spline
    unsigned int max_step_smoothing;
    // maximum time used for path simplificaton
    double max_time_soln_simpilification;
    // free joint parameters for more than 6 dof
    //std::vector<manipulator_planner_library::Robotfreeparameter> free_joint_parameters;
    std::string planner_specific_parameters_range;
};

struct EnvironmentParameters
{
    std::string input_frame_name;
    double octree_resolution;
    std::string collision_object_name;

};




// struct ConstrainedPose
// {
// 
//     base::samples::RigidBodyState target_pose;
//     manipulator_planner_library::OrientationConstraint cartesian_constraint;  // the class nanme need to changed
// 
// };




}

#endif
