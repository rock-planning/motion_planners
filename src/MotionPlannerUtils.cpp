#include <motion_planners/MotionPlanners.hpp>

#include <tinyxml2.h>
#include <boost/container_hash/hash.hpp>
#include <unordered_set>
#include <fstream>
#include <iostream>

using namespace motion_planners;

namespace
{
    // Helper function to determine base name based on reference frame and robot name
    std::string getBaseName(const std::string &reference_frame, const std::string &robot_name)
    {
        if (reference_frame == "webot_world")
        {
            return "WEBOTS_WORLD_link";
        }
        else if (reference_frame == "base_link")
        {
            if (robot_name == "kuka")
            {
                return "IIWA14_BASE_LINK_link";
            }
            if (robot_name == "vispa")
            {
                return "VISPA_BASE_LINK_link";
            }
        }
        else
        {
            return "";
        }
    }

    bool fileExists(const std::string &path)
    {
        return boost::filesystem::exists(path);
    }
}

bool MotionPlanners::getMotionPlannerConfig(motion_planners::Config &config,
                                            const std::string &config_folder_path,
                                            const std::string &robot_name,
                                            const std::string &planner_name,
                                            const std::string &solver_name,
                                            const std::string &reference_frame)
{
    // get kinematics config
    if (!getKinematicsConfig(config.planner_config.kinematics_config,
                             config_folder_path, robot_name, solver_name, reference_frame))
        return false;

    // get robot model config
    if (!getRobotModelConfig(config.planner_config.robot_model_config,
                             config_folder_path, robot_name))
        return false;

    // planner specific config (stomp_kuka or stomp_vispa)
    config.planner_config.planner_specific_config =
        config_folder_path + "/planner/" + planner_name + "_" + robot_name + ".yml";

    // planner
    if (planner_name == "trajopt")
    {
        config.planner_config.planner = motion_planners::TRAJOPT;
    }
    else if (planner_name == "ompl")
    {
        config.planner_config.planner = motion_planners::OMPL;
    }
    else
    {
        config.planner_config.planner = motion_planners::STOMP;
    }

    // Get collision detection config
    std::vector<std::string> all_links = getAllRobotLinks(config.planner_config.robot_model_config.urdf_file);

    // get environment config
    return getCollisionDetectionConfig(config.env_config,
                                       config.planner_config.robot_model_config.srdf_file,
                                       all_links, reference_frame, robot_name);
}

bool MotionPlanners::getKinematicsConfig(kinematics_library::KinematicsConfig &kinematic_config,
                                         const std::string &test_folder_path,
                                         const std::string &robot_name,
                                         const std::string &solver_name,
                                         const std::string &reference_frame)
{
    std::string urdf_path = test_folder_path + "/data/eurise_scene.urdf";
    if (!fileExists(urdf_path))
    {
        std::cout << "[getKinematicsConfig] No URDF file\n";
        return false;
    }

    kinematic_config.urdf_file = urdf_path;
    kinematic_config.solver_config_abs_path = test_folder_path + "/solver";

    // Set robot-specific parameters
    if (robot_name == "kuka")
    {
        kinematic_config.config_name = "kuka_arm";
        kinematic_config.tip_name = "IIWA14_LINK_7_link";
    }
    else if (robot_name == "vispa")
    {
        kinematic_config.config_name = "vispa_arm";
        kinematic_config.tip_name = "VISPA_LINK_6_link";
    }

    // Set base name
    kinematic_config.base_name = getBaseName(reference_frame, robot_name);

    // Set solver type and config filename
    if (solver_name == "kdl")
    {
        kinematic_config.kinematic_solver = kinematics_library::KDL;
        kinematic_config.solver_config_filename = "kdl_config.yml";
    }
    else if (solver_name == "opt")
    {
        kinematic_config.kinematic_solver = kinematics_library::OPT;
        kinematic_config.solver_config_filename = "opt_ik_config_" + robot_name + ".yml";
    }
    else
    {
        kinematic_config.kinematic_solver = kinematics_library::TRACIK;
        kinematic_config.solver_config_filename = "trac_ik_config_" + robot_name + ".yml";
    }

    return true;
}

bool MotionPlanners::getRobotModelConfig(robot_model::RobotModelConfig &robot_config,
                                         const std::string &test_folder_path,
                                         const std::string &robot_name)
{
    std::string urdf_path = test_folder_path + "/data/eurise_scene.urdf";
    if (!fileExists(urdf_path))
    {
        std::cout << "[getRobotModelConfig] No URDF file\n";
        return false;
    }

    robot_config.urdf_file = urdf_path;

    std::string srdf_file = test_folder_path + "/data/" + robot_name + ".srdf";
    if (!fileExists(srdf_file))
    {
        std::cout << "[getRobotModelConfig] No SRDF file\n";
        return false;
    }

    robot_config.srdf_file = srdf_file;
    robot_config.planning_group_name = robot_name + "_manipulator";

    return true;
}

bool MotionPlanners::reInitializeRobotModelConfig(const std::string &test_folder_path,
                                                  const std::string &robot_name,
                                                  const std::string &planner_name,
                                                  const std::string &reference_frame)
{
    std::string srdf_file = test_folder_path + "/data/" + robot_name + ".srdf";
    if (!fileExists(srdf_file))
    {
        std::cout << "[reInitializeRobotModelConfig] No SRDF file\n";
        return false;
    }

    robot_model_->setSRDFfileAbsolutePath(srdf_file);
    std::string group_name = robot_name + "_manipulator";
    robot_model_->setPlanningGroupName(group_name);

    // planner specific config (stomp_kuka or stomp_vispa)
    config_.planner_config.planner_specific_config =
        test_folder_path + "/planner/" + planner_name + "_" + robot_name + ".yml";

    // planner
    if (planner_name == "trajopt")
    {
        config_.planner_config.planner = motion_planners::TRAJOPT;
    }
    else if (planner_name == "ompl")
    {
        config_.planner_config.planner = motion_planners::OMPL;
    }
    else
    {
        config_.planner_config.planner = motion_planners::STOMP;
    }

    // Get collision detection config
    std::vector<std::string> all_links = getAllRobotLinks(robot_model_->getURDFfileAbsolutePath());

    // get environment config
    return getCollisionDetectionConfig(config_.env_config, srdf_file,
                                       all_links, reference_frame, robot_name);
}

bool MotionPlanners::getCollisionDetectionConfig(motion_planners::EnvironmentConfig &env_config,
                                                 const std::string &srdf_path,
                                                 const std::vector<std::string> &all_links,
                                                 const std::string &reference_frame,
                                                 const std::string &robot_name)
{
    // Set environment frame
    env_config.env_frame = getBaseName(reference_frame, robot_name);

    // Set collision detection config defaults
    auto &collision_config = env_config.collision_detection_config;
    collision_config.collision_library = collision_detection::FCL;
    collision_config.collision_info_type = collision_detection::MULTI_CONTACT;
    collision_config.stop_after_first_collision = true;
    collision_config.calculate_distance_information = false;
    collision_config.max_num_collision_contacts = 1;

    // Disable octree debug output
    auto &debug_config = collision_config.env_debug_config;
    debug_config.save_octree = false;
    debug_config.save_octree_filename.clear();
    debug_config.save_octree_path.clear();

    // Get enabled collision pairs from SRDF
    const StringPairSet enabled_pairs = getEnabledCollisionPairs(srdf_path);

    // Disable all pairs not explicitly enabled
    auto &disabled_pairs = env_config.disabled_collision_pair.collision_link_names;
    disabled_pairs.clear();
    for (size_t i = 0; i < all_links.size(); ++i)
    {
        for (size_t j = i + 1; j < all_links.size(); ++j)
        {
            const auto &link1 = all_links[i], &link2 = all_links[j];
            if (enabled_pairs.count({link1, link2}) == 0 &&
                enabled_pairs.count({link2, link1}) == 0) // If found keep enabled, otherwise disable
            {
                disabled_pairs.emplace_back(link1, link2);
            }
        }
    }

    // Disable collision between environment and base frame
    disabled_pairs.emplace_back("environment", env_config.env_frame);
    return true;
}

std::vector<std::string> MotionPlanners::getAllRobotLinks(const std::string &urdf_path)
{
    std::vector<std::string> link_names;
    tinyxml2::XMLDocument doc;

    // Load the URDF file
    if (doc.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        std::cout << "[MotionPlanners::getAllRobotLinks] Failed to load URDF file: " << urdf_path << std::endl;
        return link_names;
    }

    // Get the root element (<robot>)
    tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");
    if (!robot)
    {
        std::cout << "[MotionPlanners::getAllRobotLinks] URDF does not contain <robot> element." << std::endl;
        return link_names;
    }

    // Iterate over all <link> elements
    for (tinyxml2::XMLElement *link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
    {
        const char *name = link->Attribute("name");
        if (name)
            link_names.emplace_back(name);
    }

    return link_names;
}

StringPairSet MotionPlanners::getEnabledCollisionPairs(const std::string &srdf_path)
{
    StringPairSet enabled_pairs;
    tinyxml2::XMLDocument doc;

    if (doc.LoadFile(srdf_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        std::cout << "[MotionPlanners::getEnabledCollisionPairs] Error: Unable to load SRDF file " << srdf_path << std::endl;
        return enabled_pairs;
    }

    tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");
    if (!robot)
    {
        std::cout << "[MotionPlanners::getEnabledCollisionPairs] Error: No <robot> tag found in SRDF!" << std::endl;
        return enabled_pairs;
    }

    tinyxml2::XMLElement *collision_matrix = robot->FirstChildElement("collision_matrix");
    if (!collision_matrix)
    {
        std::cout << "[MotionPlanners::getEnabledCollisionPairs] Warning: No <collision_matrix> found in SRDF. Defaulting to enabling all collisions." << std::endl;
        return enabled_pairs;
    }

    for (auto *pair = collision_matrix->FirstChildElement("pair"); pair; pair = pair->NextSiblingElement("pair"))
    {
        const char *link1 = pair->Attribute("link1");
        const char *link2 = pair->Attribute("link2");

        if (link1 && link2)
            enabled_pairs.emplace(link1, link2);
    }

    return enabled_pairs;
}

void MotionPlanners::updateRobot(const base::samples::Joints &robot_status)
{
    robot_model_->updateJointGroup(robot_status);
}

/// Printing methods
void MotionPlanners::printPlannerStatus(motion_planners::PlannerStatus &planner_status)
{
    switch (planner_status.statuscode)
    {
    case motion_planners::PlannerStatus::INIT:
        std::cout << "PLANNER_INITIALIZED" << std::endl;
        break;
    case motion_planners::PlannerStatus::PATH_FOUND:
        std::cout << "PATH_FOUND" << std::endl;
        break;
    case motion_planners::PlannerStatus::NO_PATH_FOUND:
        std::cout << "NO_PATH_FOUND" << std::endl;
        break;
    case motion_planners::PlannerStatus::START_STATE_IN_COLLISION:
        std::cout << "START_STATE_IN_COLLISION" << std::endl;
        break;
    case motion_planners::PlannerStatus::GOAL_STATE_IN_COLLISION:
        std::cout << "GOAL_STATE_IN_COLLISION" << std::endl;
        break;
    case motion_planners::PlannerStatus::START_JOINTANGLES_NOT_AVAILABLE:
        std::cout << "START_JOINTANGLES_NOT_AVAILABLE" << std::endl;
        break;
    case motion_planners::PlannerStatus::GOAL_JOINTANGLES_NOT_AVAILABLE:
        std::cout << "GOAL_JOINTANGLES_NOT_AVAILABLE" << std::endl;
        break;
    case motion_planners::PlannerStatus::PLANNING_REQUEST_SUCCESS:
        std::cout << "PLANNING_REQUEST_SUCCESS" << std::endl;
        break;
    case motion_planners::PlannerStatus::CONSTRAINED_POSE_NOT_WITHIN_BOUNDS:
        std::cout << "CONSTRAINED_POSE_NOT_WITHIN_BOUNDS" << std::endl;
        break;
    case motion_planners::PlannerStatus::TIMEOUT:
        std::cout << "TIMEOUT" << std::endl;
        break;
    case motion_planners::PlannerStatus::INVALID_START_STATE:
        std::cout << "INVALID_START_STATE" << std::endl;
        break;
    case motion_planners::PlannerStatus::INVALID_GOAL_STATE:
        std::cout << "INVALID_GOAL_STATE" << std::endl;
        break;
    case motion_planners::PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
        std::cout << "UNRECOGNIZED_GOAL_TYPE" << std::endl;
        break;
    case motion_planners::PlannerStatus::APPROXIMATE_SOLUTION:
        std::cout << "APPROXIMATE_SOLUTION" << std::endl;
        break;
    case motion_planners::PlannerStatus::EXACT_SOLUTION:
        std::cout << "PATH_FOUND" << std::endl;
        break;
    case motion_planners::PlannerStatus::ROBOTMODEL_INITIALISATION_FAILED:
        std::cout << "ROBOTMODEL_INITIALISATION_FAILED" << std::endl;
        break;
    case motion_planners::PlannerStatus::PLANNER_INITIALISATION_FAILED:
        std::cout << "PLANNER_INITIALISATION_FAILED" << std::endl;
        break;
    case motion_planners::PlannerStatus::CRASH:
        std::cout << "CRASH" << std::endl;
        break;
    case motion_planners::PlannerStatus::KINEMATIC_ERROR:
    {
        switch (planner_status.kinematic_status.statuscode)
        {
        case kinematics_library::KinematicsStatus::KDL_TREE_FAILED:
            std::cout << "KDL_TREE_FAILED" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::KDL_CHAIN_FAILED:
            std::cout << "KDL_CHAIN_FAILED" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::URDF_FAILED:
            std::cout << "URDF_FAILED" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::NO_KINEMATIC_SOLVER_FOUND:
            std::cout << "NO_KINEMATIC_SOLVER_FOUND" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::IK_FOUND:
            std::cout << "IK_FOUND" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::NO_IK_SOLUTION:
            std::cout << "NO_IK_SOLUTION" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::NO_FK_SOLUTION:
            std::cout << "NO_FK_SOLUTION" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::IK_TIMEOUT:
            std::cout << "IK_TIMEOUT" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::IK_JOINTLIMITS_VIOLATED:
            std::cout << "IK_JOINTLIMITS_VIOLATED" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::NO_CONFIG_FILE:
            std::cout << "NO_KINEMATIC_CONFIG_FILE" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::CONFIG_READ_ERROR:
            std::cout << "KINEMATIC_CONFIG_READ_ERROR" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::INVALID_STATE:
            std::cout << "INVALID_KINEMATIC_STATE" << std::endl;
            break;
        case kinematics_library::KinematicsStatus::APPROX_IK_SOLUTION:
            std::cout << "IK_FOUND" << std::endl;
            break;
        default:
        {
            std::cout << "unknown Kinematics state" << planner_status.kinematic_status.statuscode << std::endl;
            throw new std::runtime_error("This kinematic status is unknown");
            break;
        }
        }
        break;
    }
    case motion_planners::PlannerStatus::INVALID:
        std::cout << "UNKNOWN_STATE" << std::endl;
        break;
    default:
    {
        LOG_ERROR("[PlannerTask]: Planner is in an unknown state. The current state value is %d", planner_status.statuscode);
        std::cout << "UNKNOWN_STATE" << std::endl;
        break;
    }
    }
}

void MotionPlanners::printPlanningGroupJoints(const std::vector<std::pair<std::string, urdf::Joint>> &joints)
{
    for (const auto &joint_pair : joints)
    {
        const auto &joint_name = joint_pair.first;
        const auto &joint = joint_pair.second;
        const auto &pose = joint.parent_to_joint_origin_transform;

        std::cout << "Joint Name: " << joint_name << "\n"
                  << "  Type: " << joint.type << "\n"
                  << "  Translation: (" << pose.position.x << ", "
                  << pose.position.y << ", "
                  << pose.position.z << ")\n"
                  << "  Rotation: (" << pose.rotation.x << ", "
                  << pose.rotation.y << ", "
                  << pose.rotation.z << ", "
                  << pose.rotation.w << ")\n"
                  << "  Child Link: " << joint.child_link_name << "\n";
    }
}

void MotionPlanners::printIKSolution(const std::vector<base::commands::Joints> &ik_solution)
{
    std::cout << "Size of IK solutions: " << ik_solution.size() << "\n";
    for (size_t i = 0; i < ik_solution.size(); ++i)
    {
        std::cout << "IK Solution " << i + 1 << ":\n  Joint Positions: ";
        for (const auto &joint : ik_solution[i].elements)
        {
            std::cout << joint.position << " ";
        }
        std::cout << std::endl;
    }
}

bool MotionPlanners::ExcessiveJointMotion(const base::JointsTrajectory &traj, double max_angle_rad)
{
    if (LargeJointMotionOverWholePath(traj, max_angle_rad))
        return true;

    for (size_t j = 0; j < traj.getNumberOfJoints(); ++j)
    {
        for (size_t t = 1; t < traj.getTimeSteps(); ++t)
        {
            double delta = std::abs(traj.elements[j][t].position - traj.elements[j][t - 1].position);
            if (delta > max_angle_rad)
            {
                std::cout << "Joint = " << j << "; jump = " << delta << " rad between t = "
                          << t - 1 << " and t = " << t << std::endl;
                // LOG_DEBUG("Joint = %d; jumps = %d rad between t = %d; and t = %d", j, delta, t - 1, t);
                return true;
            }
        }
    }
    return false;
}

bool MotionPlanners::LargeJointMotionOverWholePath(const base::JointsTrajectory &traj,
                                                   double max_angle_rad)
{
    size_t time_steps = traj.getTimeSteps();
    size_t num_joints = traj.getNumberOfJoints();

    for (size_t j = 0; j < num_joints; ++j)
    {
        double pos_start = traj.elements[j][0].position;
        double pos_end = traj.elements[j][time_steps - 1].position;
        double total_motion = std::abs(pos_end - pos_start);

        if (total_motion > max_angle_rad)
        {
            std::cout << "Joint = " << j << "; moves = " << total_motion << " rad from start to end (exceeds " << max_angle_rad << " rad)" << std::endl;
            // LOG_DEBUG("Joint = %d; moves = %d rad from start to end (exceeds %d rad)", j, total_motion, max_angle_rad);
            return true;
        }
    }
    return false;
}

// Create an iterate function