#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <utility>
#include <string>
#include <vector>
#include <kinematics_library/KinematicsConfig.hpp>
#include <collision_detection/CollisionConfig.hpp>


namespace motion_planners
{

struct CollisionInformation
{
    std::pair<std::string,std::string>  collision_pair;
    std::vector <std::pair<std::string,std::string> > collision_pair_names;
};


struct ModelObject
{
    ModelObject():
        operation(collision_detection::RESET),
        model_type(collision_detection::UNDEFINED),
        object_path(""), object_name("") {}

    // this variable says whether the model object should be added or removed
    collision_detection::Operation operation;
    // model type: primitives or mesh
    collision_detection::ModelTypes model_type;
    // primitive types: box, cylinder, sphere
    collision_detection::PrimitiveObject primitive_object;
    // mesh file path
    std::string object_path;
    // object name, please give an object name
    std::string object_name;
    // attach the model object to the given link name
    std::string attach_link_name;
    // relative pose of the model object w.r.t to attach_link
    base::Pose relative_pose;
};

    /**
 *  @struct RobotModelParameters.
 * @brief This struct contains parameters used in the robot model.
 */
struct RobotModelParameters
{
    // srdf file abs path
    std::string srdf_file;
    // urdf file abs path
    std::string urdf_file;
    // planning group
    std::string planning_group_name;
};

enum PlannerLibrary
{
    STOMP, OMPL, TRAJOPT
};

struct PlannerConfig
{
    kinematics_library::KinematicsConfig kinematics_config; 
    RobotModelParameters robot_model_config;
    std::string planner_specific_config;
    double distance;
    enum PlannerLibrary planner;
};


struct EnvironmentConfig
{
    std::string env_frame;
    double octree_resolution;
    std::string env_object_name;
    bool do_self_filter;
    std::vector <std::pair<std::string,std::string> > disabled_collision_pair;
    
};

struct Config
{
    PlannerConfig planner_config;
    EnvironmentConfig env_config;    
};

}// end motion_planners
#endif // PLANNERSTATUS_HPP

