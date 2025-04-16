#include <vector>
#include <string>

#include <motion_planners/MotionPlanners.hpp>

#include <stdio.h>
#include <tinyxml2.h>
#include <unordered_set>
#include <boost/container_hash/hash.hpp> // Ensure boost::hash is fully defined
#include <fstream>

using namespace motion_planners;

kinematics_library::KinematicsConfig MotionPlanners::getKinematicsConfig(const std::string &test_folder_path)
{
    kinematics_library::KinematicsConfig config;

    config.config_name = "kuka_arm";
    config.base_name = "WEBOTS_WORLD_link";
    config.tip_name = "IIWA14_LINK_7_link";
    config.urdf_file = test_folder_path + "/data/eu-rise/eurise_scene.urdf";
    config.kinematic_solver = kinematics_library::TRACIK; // kinematics_library::KDL; kinematics_library::TRACIK; kinematics_library::OPT;
    config.solver_config_abs_path = test_folder_path + "/config";
    config.solver_config_filename = "trac_ik_config.yml"; // kdl_config.yml; trac_ik_config.yml; opt_ik_config.yml;

    return config;
}

robot_model::RobotModelConfig MotionPlanners::getRobotModelConfig(const std::string &test_folder_path)
{
    robot_model::RobotModelConfig config;
    // srdf file abs path
    config.srdf_file = test_folder_path + "/data/eu-rise/kuka.srdf";
    // urdf file abs path
    config.urdf_file = test_folder_path + "/data/eu-rise/eurise_scene.urdf";
    // planning group
    config.planning_group_name = "kuka_manipulator";

    return config;
};

std::vector<std::string> MotionPlanners::getAllRobotLinks(const std::string &urdf_path)
{
    std::vector<std::string> link_names;
    tinyxml2::XMLDocument doc;

    // Load the URDF file
    if (doc.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        std::cerr << "Failed to load URDF file: " << urdf_path << std::endl;
        return link_names;
    }

    // Get the root element (<robot>)
    tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");
    if (!robot)
    {
        std::cerr << "URDF does not contain <robot> element." << std::endl;
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

std::unordered_set<std::pair<std::string, std::string>, boost::hash<std::pair<std::string, std::string>>> MotionPlanners::getEnabledCollisionPairs(const std::string &srdf_path)
{
    std::unordered_set<std::pair<std::string, std::string>, boost::hash<std::pair<std::string, std::string>>> enabled_pairs;
    tinyxml2::XMLDocument doc;

    if (doc.LoadFile(srdf_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        std::cout << "Error: Unable to load SRDF file " << srdf_path << std::endl;
        return enabled_pairs;
    }

    tinyxml2::XMLElement *robot = doc.FirstChildElement("robot");
    if (!robot)
    {
        std::cout << "Error: No <robot> tag found in SRDF!" << std::endl;
        return enabled_pairs;
    }

    tinyxml2::XMLElement *collision_matrix = robot->FirstChildElement("collision_matrix");
    if (!collision_matrix)
    {
        std::cout << "Warning: No <collision_matrix> found in SRDF. Defaulting to enabling all collisions." << std::endl;
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

motion_planners::EnvironmentConfig MotionPlanners::getCollisionDetectionConfig(const std::string &srdf_path,
                                                                               const std::vector<std::string> &all_links)
{
    motion_planners::EnvironmentConfig config;
    config.env_frame = "IIWA14_BASE_LINK_link";
    config.collision_detection_config.collision_library = collision_detection::FCL;
    config.collision_detection_config.collision_info_type = collision_detection::MULTI_CONTACT;
    config.collision_detection_config.stop_after_first_collision = true;
    config.collision_detection_config.calculate_distance_information = false;
    config.collision_detection_config.max_num_collision_contacts = 1;
    config.collision_detection_config.env_debug_config.save_octree = false;
    config.collision_detection_config.env_debug_config.save_octree_filename = "";
    config.collision_detection_config.env_debug_config.save_octree_path = "";
    // Get enabled collision pairs from SRDF
    auto enabled_pairs = getEnabledCollisionPairs(srdf_path);

    // Open a file to write the disabled collision pairs
    std::ofstream output_file("disabled_collisions_kuka.xml");

    if (!output_file.is_open())
    {
        std::cout << "Error: Unable to open file for writing!" << std::endl;
    }

    // Iterate through all possible pairs and disable everything except enabled ones
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
                collision_detection::CollisionLinkName disabled_collision(link1, link2);
                // Write to the file instead of console output
                output_file << "<disable_collisions link1=\"" << link1
                            << "\" link2=\"" << link2
                            << "\" reason=\"Never\" />\n";
                config.disabled_collision_pair.collision_link_names.push_back(disabled_collision);
            }
        }
    }

    // output_file.close();

    config.env_object_name = "environment";
    collision_detection::CollisionLinkName disabled_collision("environment", "IIWA14_BASE_LINK_link"); // WEBOTS_WORLD_link
    config.disabled_collision_pair.collision_link_names.push_back(disabled_collision);

    return config;
}

motion_planners::Config MotionPlanners::getMotionPlannerConfig(const std::string &config_folder_path)
{
    motion_planners::Config config;

    // get kinematics config
    config.planner_config.kinematics_config = getKinematicsConfig(config_folder_path);
    // get robot model config
    config.planner_config.robot_model_config = getRobotModelConfig(config_folder_path);
    // planner specific config
    config.planner_config.planner_specific_config = config_folder_path + "/config/stomp_kuka.yml"; // stomp.yml
    // planner
    config.planner_config.planner = motion_planners::STOMP; // motion_planners::STOMP;

    std::vector<std::string> all_links = getAllRobotLinks(config.planner_config.robot_model_config.urdf_file);

    // get collision detection config
    config.env_config = getCollisionDetectionConfig(config.planner_config.robot_model_config.srdf_file, all_links);

    return config;
}

// Create an iterate function