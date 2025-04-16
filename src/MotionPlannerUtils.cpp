#include <motion_planners/MotionPlanners.hpp>

#include <stdio.h>
#include <tinyxml2.h>
#include <unordered_set>
#include <boost/container_hash/hash.hpp> // Ensure boost::hash is fully defined
#include <fstream>

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
            return (robot_name == "kuka") ? "IIWA14_BASE_LINK_link" : (robot_name == "vispa") ? "VISPA_BASE_LINK_link"
                                                                                              : "";
        }
        return "";
    }
}

bool MotionPlanners::getMotionPlannerConfig(motion_planners::Config &config,
                                            const std::string &config_folder_path,
                                            const std::string &robot_name,
                                            const std::string &planner_name,
                                            const std::string &solver_name,
                                            const std::string &reference_frame)
{
    bool result;

    // get kinematics config
    result = getKinematicsConfig(config.planner_config.kinematics_config,
                                 config_folder_path, robot_name, solver_name, reference_frame);

    if (result == false)
    {
        return result;
    }

    // get robot model config
    result = getRobotModelConfig(config.planner_config.robot_model_config,
                                 config_folder_path, robot_name);

    if (result == false)
    {
        return result;
    }

    // planner specific config (stomp_kuka or stomp_vispa)
    config.planner_config.planner_specific_config = config_folder_path + "/config/" +
                                                    planner_name + "_" + robot_name + ".yml";

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
    config.env_config = getCollisionDetectionConfig(
        config.planner_config.robot_model_config.srdf_file, all_links, reference_frame, robot_name);

    return true;
}

bool MotionPlanners::getKinematicsConfig(kinematics_library::KinematicsConfig &kinematic_config,
                                         const std::string &test_folder_path,
                                         const std::string &robot_name,
                                         const std::string &solver_name,
                                         const std::string &reference_frame)
{
    std::string urdf_path = test_folder_path + "/data/eu-rise/eurise_scene.urdf";
    std::cout << "urdf_path " << urdf_path << std::endl;
    if (boost::filesystem::exists(urdf_path))
    {
        kinematic_config.urdf_file = urdf_path;
    }
    else
    {
        std::cout << "[MotionPlanners::getKinematicsConfig] No URDF file" << std::endl;
        return false;
    }

    kinematic_config.solver_config_abs_path = test_folder_path + "/config";

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
        kinematic_config.solver_config_filename = "opt_ik_config.yml";
    }
    else
    {
        kinematic_config.kinematic_solver = kinematics_library::TRACIK;
        kinematic_config.solver_config_filename = "trac_ik_config.yml";
    }

    return true;
}

bool MotionPlanners::getRobotModelConfig(robot_model::RobotModelConfig &robot_config,
                                         const std::string &test_folder_path,
                                         const std::string &robot_name)
{
    std::string urdf_path = test_folder_path + "/data/eu-rise/eurise_scene.urdf";
    if (boost::filesystem::exists(urdf_path))
    {
        robot_config.urdf_file = urdf_path;
    }
    else
    {
        std::cout << "[MotionPlanners::getRobotModelConfig] No URDF file" << std::endl;
        return false;
    }

    // Set robot-specific parameters
    if (robot_name == "kuka")
    {
        std::string srdf_path = test_folder_path + "/data/eu-rise/kuka.srdf";
        if (boost::filesystem::exists(srdf_path))
        {
            robot_config.srdf_file = srdf_path;
        }
        else
        {
            std::cout << "[MotionPlanners::getRobotModelConfig] No SRDF file" << std::endl;
            return false;
        }
        robot_config.planning_group_name = "kuka_manipulator";
    }
    else if (robot_name == "vispa")
    {
        robot_config.srdf_file = test_folder_path + "/data/eu-rise/vispa.srdf";
        robot_config.planning_group_name = "vispa_manipulator";
    }

    return true;
}

motion_planners::EnvironmentConfig MotionPlanners::getCollisionDetectionConfig(const std::string &srdf_path,
                                                                               const std::vector<std::string> &all_links,
                                                                               const std::string &reference_frame,
                                                                               const std::string &robot_name)
{
    motion_planners::EnvironmentConfig env_config;

    // Set environment frame
    env_config.env_frame = getBaseName(reference_frame, robot_name);

    env_config.collision_detection_config.collision_library = collision_detection::FCL;
    env_config.collision_detection_config.collision_info_type = collision_detection::MULTI_CONTACT;
    env_config.collision_detection_config.stop_after_first_collision = true;
    env_config.collision_detection_config.calculate_distance_information = false;
    env_config.collision_detection_config.max_num_collision_contacts = 1;
    env_config.collision_detection_config.env_debug_config.save_octree = false;
    env_config.collision_detection_config.env_debug_config.save_octree_filename = "";
    env_config.collision_detection_config.env_debug_config.save_octree_path = "";
    // Get enabled collision pairs from SRDF
    StringPairSet enabled_pairs = getEnabledCollisionPairs(srdf_path);

    // Disable all pairs except enabled ones
    for (size_t i = 0; i < all_links.size(); ++i)
    {
        for (size_t j = i + 1; j < all_links.size(); ++j)
        {
            std::string link1 = all_links[i];
            std::string link2 = all_links[j];

            // If this pair is NOT in the enabled list, disable it
            if (enabled_pairs.find({link1, link2}) == enabled_pairs.end() &&
                enabled_pairs.find({link2, link1}) == enabled_pairs.end())
            {
                env_config.disabled_collision_pair.collision_link_names.emplace_back(link1, link2);
            }
        }
    }

    // Disable collision between environment and base frame
    collision_detection::CollisionLinkName disabled_collision("environment", env_config.env_frame);
    env_config.disabled_collision_pair.collision_link_names.push_back(disabled_collision);

    return env_config;
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
        {
            link_names.push_back(name);
        }
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

    tinyxml2::XMLElement *pair = collision_matrix->FirstChildElement("pair");
    while (pair)
    {
        const char *link1 = pair->Attribute("link1");
        const char *link2 = pair->Attribute("link2");

        if (link1 && link2)
        {
            enabled_pairs.insert(std::make_pair(std::string(link1), std::string(link2)));
        }

        pair = pair->NextSiblingElement("pair");
    }

    return enabled_pairs;
}

// Create an iterate function