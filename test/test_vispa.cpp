#include <motion_planners/MotionPlanners.hpp>
// #include <tinyxml2.h>
// #include <unordered_set>
// #include <urdf_parser/urdf_parser.h> // If using urdfdom
// #include <urdf_model/model.h> // If using urdfdom_headers
// #include <urdf_model/link.h> 
// #include <urdf_model/joint.h>
// #include <boost/container_hash/hash.hpp>  // Ensure boost::hash is fully defined
// #include <fstream>

using namespace motion_planners;

// std::vector<std::pair<std::string, std::string>> getDisabledCollisionPairs(const std::string& srdf_path)
// {
//     std::vector<std::pair<std::string, std::string>> disabled_pairs;
//     tinyxml2::XMLDocument doc;
    
//     if (doc.LoadFile(srdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
//         std::cout << "Error: Unable to load SRDF file " << srdf_path << std::endl;
//         return disabled_pairs;
//     }

//     tinyxml2::XMLElement* robot = doc.FirstChildElement("robot");
//     if (!robot) {
//         std::cout << "Error: No <robot> tag found in SRDF!" << std::endl;
//         return disabled_pairs;
//     }

//     tinyxml2::XMLElement* collision_matrix = robot->FirstChildElement("collision_matrix");
//     if (!collision_matrix) {
//         std::cout << "Warning: No <collision_matrix> found in SRDF. Defaulting to enabled collisions." << std::endl;
//         return disabled_pairs;
//     }

//     tinyxml2::XMLElement* pair = collision_matrix->FirstChildElement("pair");
//     while (pair) {
//         const char* link1 = pair->Attribute("link1");
//         const char* link2 = pair->Attribute("link2");

//         if (link1 && link2) {
//             disabled_pairs.emplace_back(link1, link2);
//         }

//         pair = pair->NextSiblingElement("pair");
//     }

//     return disabled_pairs;
// }

// std::unordered_set<std::pair<std::string, std::string>, boost::hash<std::pair<std::string, std::string>>> getEnabledCollisionPairs(const std::string& srdf_path)
// {
//     std::unordered_set<std::pair<std::string, std::string>, boost::hash<std::pair<std::string, std::string>>> enabled_pairs;
//     tinyxml2::XMLDocument doc;

//     if (doc.LoadFile(srdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
//         std::cout << "Error: Unable to load SRDF file " << srdf_path << std::endl;
//         return enabled_pairs;
//     }

//     tinyxml2::XMLElement* robot = doc.FirstChildElement("robot");
//     if (!robot) {
//         std::cout << "Error: No <robot> tag found in SRDF!" << std::endl;
//         return enabled_pairs;
//     }

//     tinyxml2::XMLElement* collision_matrix = robot->FirstChildElement("collision_matrix");
//     if (!collision_matrix) {
//         std::cout << "Warning: No <collision_matrix> found in SRDF. Defaulting to enabling all collisions." << std::endl;
//         return enabled_pairs;
//     }

//     tinyxml2::XMLElement* pair = collision_matrix->FirstChildElement("pair");
//     while (pair) {
//         const char* link1 = pair->Attribute("link1");
//         const char* link2 = pair->Attribute("link2");

//         if (link1 && link2) {
//             enabled_pairs.insert({std::string(link1), std::string(link2)});
//         }

//         pair = pair->NextSiblingElement("pair");
//     }

//     return enabled_pairs;
// }

// std::vector<std::string> getAllRobotLinks(const std::string& urdf_path) {
//     std::vector<std::string> link_names;
//     tinyxml2::XMLDocument doc;

//     // Load the URDF file
//     if (doc.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
//         std::cerr << "Failed to load URDF file: " << urdf_path << std::endl;
//         return link_names;
//     }

//     // Get the root element (<robot>)
//     tinyxml2::XMLElement* robot = doc.FirstChildElement("robot");
//     if (!robot) {
//         std::cerr << "URDF does not contain <robot> element." << std::endl;
//         return link_names;
//     }

//     // Iterate over all <link> elements
//     for (tinyxml2::XMLElement* link = robot->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
//         const char* name = link->Attribute("name");
//         if (name) {
//             link_names.push_back(name);
//         }
//     }

//     return link_names;
// }


motion_planners::EnvironmentConfig getCollisionDetectionConfig()
{
    motion_planners::EnvironmentConfig config;
    config.env_frame = "VISPA_BASE_LINK_link";
    // config.env_frame = "WEBOTS_WORLD_link";
    config.collision_detection_config.collision_library = collision_detection::FCL;
    config.collision_detection_config.collision_info_type = collision_detection::MULTI_CONTACT;
    config.collision_detection_config.stop_after_first_collision = true;
    config.collision_detection_config.calculate_distance_information = false;
    config.collision_detection_config.max_num_collision_contacts = 1;
    config.collision_detection_config.env_debug_config.save_octree = false;
    config.collision_detection_config.env_debug_config.save_octree_filename = "";
    config.collision_detection_config.env_debug_config.save_octree_path = "";
    config.env_object_name = "environment";
    collision_detection::CollisionLinkName disabled_collision("environment", "VISPA_BASE_LINK_link");
    config.disabled_collision_pair.collision_link_names.push_back(disabled_collision);
    return config;
}

// motion_planners::EnvironmentConfig getCollisionDetectionConfig(const std::string& srdf_path)
// {
//     motion_planners::EnvironmentConfig config;
//     config.env_frame = "VISPA_BASE_LINK_link";
//     config.collision_detection_config.collision_library = collision_detection::FCL;
//     config.collision_detection_config.collision_info_type = collision_detection::MULTI_CONTACT;
//     config.collision_detection_config.stop_after_first_collision = true;
//     config.collision_detection_config.calculate_distance_information = false;
//     config.collision_detection_config.max_num_collision_contacts = 1;
//     config.collision_detection_config.env_debug_config.save_octree = false;
//     config.collision_detection_config.env_debug_config.save_octree_filename = "";
//     config.collision_detection_config.env_debug_config.save_octree_path = "";
//     // Get disabled collisions from SRDF
//     auto disabled_pairs = getDisabledCollisionPairs(srdf_path);
//     for (const auto& pair : disabled_pairs)
//     {
//         collision_detection::CollisionLinkName disabled_collision(pair.first, pair.second);
//         config.disabled_collision_pair.collision_link_names.push_back(disabled_collision);
//     }
//     config.env_object_name = "environment";
//     collision_detection::CollisionLinkName disabled_collision("environment", "VISPA_BASE_LINK_link");
//     config.disabled_collision_pair.collision_link_names.push_back(disabled_collision);

//     return config;
// }

// motion_planners::EnvironmentConfig getCollisionDetectionConfig(const std::string& srdf_path, 
//                                                                const std::vector<std::string>& all_links)
// {
//     motion_planners::EnvironmentConfig config;
//     config.env_frame = "VISPA_BASE_LINK_link";
//     config.collision_detection_config.collision_library = collision_detection::FCL;
//     config.collision_detection_config.collision_info_type = collision_detection::MULTI_CONTACT;
//     config.collision_detection_config.stop_after_first_collision = true;
//     config.collision_detection_config.calculate_distance_information = false;
//     config.collision_detection_config.max_num_collision_contacts = 1;
//     config.collision_detection_config.env_debug_config.save_octree = false;
//     config.collision_detection_config.env_debug_config.save_octree_filename = "";
//     config.collision_detection_config.env_debug_config.save_octree_path = "";
//     // Get enabled collision pairs from SRDF
//     auto enabled_pairs = getEnabledCollisionPairs(srdf_path);

//     // // Open a file to write the disabled collision pairs
//     // std::ofstream output_file("disabled_collisions.xml");

//     // if (!output_file.is_open()) {
//     //     std::cout << "Error: Unable to open file for writing!" << std::endl;
//     // }

//     // Iterate through all possible pairs and disable everything except enabled ones
//     for (size_t i = 0; i < all_links.size(); ++i) {
//         for (size_t j = i + 1; j < all_links.size(); ++j) {
//             std::string link1 = all_links[i];
//             std::string link2 = all_links[j];

//             // If this pair is NOT in the enabled list, disable it
//             if (enabled_pairs.find({link1, link2}) == enabled_pairs.end() &&
//                 enabled_pairs.find({link2, link1}) == enabled_pairs.end()) 
//             {
//                 collision_detection::CollisionLinkName disabled_collision(link1, link2);
//                 // // Write to the file instead of console output
//                 // output_file << "<disable_collisions link1=\"" << link1
//                 //             << "\" link2=\"" << link2
//                 //             << "\" reason=\"Never\" />\n";
//                 config.disabled_collision_pair.collision_link_names.push_back(disabled_collision);
//             }
//         }
//     }

//     // output_file.close();

//     return config;
// }


kinematics_library::KinematicsConfig getKinematicsConfig(std::string test_folder_path)
{
    kinematics_library::KinematicsConfig config;

    config.config_name = "vispa_arm";
    config.base_name = "VISPA_BASE_LINK_link";
    config.tip_name = "VISPA_LINK_6_link";
    config.urdf_file = test_folder_path + "./data/eu-rise/eurise_scene.urdf";
    config.kinematic_solver = kinematics_library::KDL;
    config.solver_config_abs_path = test_folder_path + "./config";
    config.solver_config_filename = "kdl_config.yml";

    return config;
}

robot_model::RobotModelConfig getRobotModelConfig(std::string test_folder_path)
{
    robot_model::RobotModelConfig config;
    // srdf file abs path
    config.srdf_file = test_folder_path + "./data/eu-rise/vispa.srdf";
    // urdf file abs path
    config.urdf_file = test_folder_path + "./data/eu-rise/eurise_scene.urdf";
    // planning group
    config.planning_group_name = "vispa_manipulator";

    return config;
};

motion_planners::Config getMotionPlannerConfig(std::string test_folder_path)
{
    motion_planners::Config config;

    // get kinematics config
    config.planner_config.kinematics_config = getKinematicsConfig(test_folder_path);
    // get robot model config
    config.planner_config.robot_model_config = getRobotModelConfig(test_folder_path);
    // planner specific config
    config.planner_config.planner_specific_config = test_folder_path + "./config/stomp_vispa.yml"; // stomp.yml
    // planner
    config.planner_config.planner = motion_planners::STOMP; // motion_planners::STOMP;
    config.env_config = getCollisionDetectionConfig();

    // std::vector<std::string> all_links = getAllRobotLinks(config.planner_config.robot_model_config.urdf_file);

    // get collision detection config
    // config.env_config = getCollisionDetectionConfig(test_folder_path + "./data/eu-rise/vispa.srdf");
    // config.env_config = getCollisionDetectionConfig(config.planner_config.robot_model_config.srdf_file, all_links);
    return config;
}

base::samples::Joints convertToBaseJoints(const std::vector<double> &data)
{
    base::samples::Joints joint_values;
    joint_values.names = {"VISPA_LINK_1_joint", "VISPA_LINK_2_joint", "VISPA_LINK_3_joint", "VISPA_LINK_4_joint", "VISPA_LINK_5_joint", "VISPA_LINK_6_joint"};
    joint_values.elements.resize(joint_values.names.size());
    assert(joint_values.size() == data.size());
    for (size_t i = 0; i < data.size(); i++)
    {
        // std::cout << "i = " << i << "; " << data[i] << std::endl;
        joint_values.elements[i].position = data[i];
    }

    return joint_values;
}

void printTrajectory(const base::JointsTrajectory &traj)
{
    std::cout << "Number of timestep :" << traj.getTimeSteps() << ". Number of joints = " << traj.getNumberOfJoints() << std::endl;
    for (size_t i = 0; i < traj.getTimeSteps(); i++)
    {
        for (size_t j = 0; j < traj.elements.size(); j++)
        {
            std::cout << traj.elements[j][i].position << ", ";
        }
        std::cout << std::endl;
    }
}

void printPlannerStatus(motion_planners::PlannerStatus &planner_status)
{
    switch (planner_status.statuscode)
    {
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

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char *argv[])
{
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    std::cout << "!             Test function for testing motion planners          !\n";
    std::cout << "!./test_motion_planners absolute_path_to_test_folder             !\n";
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n";

    if (argc != 2)
    {
        std::cout << "The test function expects the absolute path to the test folder" << std::endl;
        return 0;
    }

    std::string test_folder_path = argv[1];
    std::cout << "The given absolute path: " << test_folder_path << std::endl;

    // get planner config
    motion_planners::Config config = getMotionPlannerConfig(test_folder_path);

    // create motion planner object
    motion_planners::MotionPlanners planner(config);
    // initialise the planner
    PlannerStatus planner_status;
    if (!planner.initialize(planner_status))
    {
        std::cout << "Motion planner failed at initialization. Refer to planner status to get the error information" << std::endl;
        printPlannerStatus(planner_status);
    }

    // assign planning request
    std::vector<double> start_vec_values = {0.0863422, 0.063656, -0.0141889, 0.0813069, 0.0775262, -0.00784293};
    base::samples::Joints start_joint_values = convertToBaseJoints(start_vec_values);
    std::vector<double> target_vec_values = {1.14037, 0.769373, 1.80815, 0.188559, 0.921982, -2.0275};
    base::samples::Joints target_joint_values = convertToBaseJoints(target_vec_values);

    if (planner.assignPlanningRequest(start_joint_values, target_joint_values, planner_status))
    {
        // plan only if the planning request is successful
        planner.setStartAndGoal(); // this function will initialize the start and goal for the planner
        double solving_time = 0.0;
        base::JointsTrajectory solution;
        if (planner.solve(solution, planner_status, solving_time))
        {
            std::cout << "Path Found" << std::endl;
            printTrajectory(solution);
        }
        else
        {
            std::cout << "No Path Found. Refer to planner status to get the error information" << std::endl;
            printPlannerStatus(planner_status);
        }
    }
    else
    {
        std::cout << "Assigning planning request failed. Refer to planner status to get the error information" << std::endl;
        printPlannerStatus(planner_status);
        collision_detection::CollisionLinksName collided_objects = planner.getCollidedObjectsNames();
        for (const auto &collision_name : collided_objects.collision_link_names)
            {
                std::cout << "Collided Object 1: " << collision_name.link_1 << ", Collided Object 2: " << collision_name.link_2 << std::endl;
            }
    }
    return 0;
}
