#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <utility>
#include <string>
#include <vector>
#include <kinematics_library/KinematicsConfig.hpp>
#include <collision_detection/CollisionConfig.hpp>
#include <robot_model/RobotModelConfig.hpp>

namespace motion_planners
{

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

enum PlannerLibrary
{
    STOMP, OMPL, TRAJOPT
};

struct PlannerConfig
{
    kinematics_library::KinematicsConfig kinematics_config;
    robot_model::RobotModelConfig robot_model_config;
    std::string planner_specific_config;
    enum PlannerLibrary planner;
};

struct DualArmPlannerConfig
{    
    kinematics_library::KinematicsConfig active_chain_kin_config;    // active arm 
    kinematics_library::KinematicsConfig passive_chain_kin_config;   // passive arm
    robot_model::RobotModelConfig robot_model_config;
    std::string planner_specific_config;
    enum PlannerLibrary planner;
};

struct EnvironmentConfig
{
    collision_detection::CollisionDetectionConfig collision_detection_config;
    // attach the environment to this frame
    std::string env_frame;
    // name of the environment
    std::string env_object_name;
    // disable collision pair between the environment with the robot's link    
    collision_detection::CollisionLinksName disabled_collision_pair;
};

struct Config
{
    PlannerConfig planner_config;
    EnvironmentConfig env_config;
};



}// end motion_planners
#endif // PLANNERSTATUS_HPP

