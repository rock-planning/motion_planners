#include <motion_planners/MotionPlanners.hpp>

#include <tinyxml2.h>
#include <boost/container_hash/hash.hpp>
#include <boost/filesystem.hpp>
#include <unordered_set>
#include <fstream>
#include <iostream>

using namespace motion_planners;
// using namespace tinyxml2;
// using namespace std;

namespace
{
    // Helper function to determine base name based on reference frame and robot name
    std::string getBaseName(motion_planners::RobotConfig &robot_links,
                            const std::string &reference_frame, const std::string &robot_name)
    {
        if (reference_frame == "webot_world")
        {
            return robot_links.world.name;
        }
        else if (reference_frame == "base_link")
        {
            if (robot_name == "kuka")
            {
                return robot_links.kuka.base;
            }
            else if (robot_name == "vispa")
            {
                return robot_links.vispa.base;
            }
        }
        else
        {
            return "";
        }
        return "";
    }

    bool fileExists(const std::string &path)
    {
        return boost::filesystem::exists(path);
    }
}

bool MotionPlanners::getMotionPlannerConfig(motion_planners::Config &config,
                                            const std::string &robot_links_str,
                                            const std::string &config_folder_path,
                                            const std::string &urdf_file,
                                            const std::string &robot_name,
                                            const std::string &planner_name,
                                            const std::string &solver_name,
                                            const std::string &reference_frame,
                                            const int &num_waypoints)
{
    this->num_waypoints = num_waypoints;
    this->robot_links.load_from_yaml(robot_links_str);
    // Create SRDF files
    generateSRDFFiles(urdf_file, "kuka_manipulator", this->robot_links.kuka.base, this->robot_links.kuka.ee, this->robot_links.kuka.j0, config_folder_path + "kuka.srdf");
    generateSRDFFiles(urdf_file, "vispa_manipulator", this->robot_links.vispa.base, this->robot_links.vispa.ee, this->robot_links.vispa.j0, config_folder_path + "vispa.srdf");

    // get kinematics config
    if (!getKinematicsConfig(config.planner_config.kinematics_config, config_folder_path,
                             urdf_file, robot_name, solver_name, reference_frame))
        return false;

    // get robot model config
    if (!getRobotModelConfig(config.planner_config.robot_model_config, config_folder_path,
                             urdf_file, robot_name))
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
                                         const std::string &config_folder_path,
                                         const std::string &urdf_file,
                                         const std::string &robot_name,
                                         const std::string &solver_name,
                                         const std::string &reference_frame)
{
    if (!fileExists(urdf_file))
    {
        std::cout << "[getKinematicsConfig] No URDF file\n";
        return false;
    }

    kinematic_config.urdf_file = urdf_file;
    kinematic_config.solver_config_abs_path = config_folder_path + "/solver";

    // Set robot-specific parameters
    if (robot_name == "kuka")
    {
        kinematic_config.config_name = "kuka_arm";
        kinematic_config.tip_name = this->robot_links.kuka.ee;
    }
    else if (robot_name == "vispa")
    {
        kinematic_config.config_name = "vispa_arm";
        kinematic_config.tip_name = this->robot_links.vispa.ee;
    }

    // Set base name
    kinematic_config.base_name = getBaseName(this->robot_links, reference_frame, robot_name);

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
                                         const std::string &config_folder_path,
                                         const std::string &urdf_file,
                                         const std::string &robot_name)
{
    if (!fileExists(urdf_file))
    {
        std::cout << "[getRobotModelConfig] No URDF file\n";
        return false;
    }

    robot_config.urdf_file = urdf_file;

    // std::string srdf_folder_path = extractDirectory(urdf_file);
    std::string srdf_file = config_folder_path + robot_name + ".srdf";

    if (!fileExists(srdf_file))
    {
        std::cout << "[getRobotModelConfig] No SRDF file\n";
        return false;
    }

    robot_config.srdf_file = srdf_file;
    robot_config.planning_group_name = robot_name + "_manipulator";

    return true;
}

bool MotionPlanners::reInitializeRobotModelConfig(const std::string &config_folder_path,
                                                  const std::string &robot_name,
                                                  const std::string &planner_name,
                                                  const std::string &reference_frame)
{
    std::string srdf_file = config_folder_path + robot_name + ".srdf";

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
        config_folder_path + "/planner/" + planner_name + "_" + robot_name + ".yml";

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
    env_config.env_frame = getBaseName(this->robot_links, reference_frame, robot_name);

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
                // std::cout << "Joint = " << j << "; jump = " << delta << " rad between t = "
                //           << t - 1 << " and t = " << t << std::endl;
                LOG_DEBUG("Joint = %d; jumps = %d rad between t = %d; and t = %d", j, delta, t - 1, t);
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
            // std::cout << "Joint = " << j << "; moves = " << total_motion << " rad from start to end (exceeds " << max_angle_rad << " rad)" << std::endl;
            LOG_DEBUG("Joint = %d; moves = %d rad from start to end (exceeds %d rad)", j, total_motion, max_angle_rad);
            return true;
        }
    }
    return false;
}

std::string MotionPlanners::extractDirectory(const std::string &filepath)
{
    size_t pos = filepath.find_last_of('/');
    if (pos != std::string::npos)
    {
        return filepath.substr(0, pos + 1); // Include trailing slash
    }
    else
    {
        return ""; // No slash found, return empty or handle differently
    }
}

/**
 * SRDF Functions
 */

std::string MotionPlanners::prettifyXML(const std::string &xmlContent)
{
    tinyxml2::XMLDocument doc;

    doc.Parse(xmlContent.c_str());

    tinyxml2::XMLPrinter printer(nullptr, false, 2); // indent = 2 spaces
    doc.Print(&printer);
    return std::string(printer.CStr());
}

// std::vector<std::string> MotionPlanners::getLinksFromChain(const std::vector<Joint> &joints, const std::string &base, const std::string &tip)
// {
//     std::vector<std::string> chain;
//     std::string current = tip;
//     while (current != base)
//     {
//         bool found = false;
//         for (const auto &joint : joints)
//         {
//             if (joint.child == current)
//             {
//                 chain.push_back(current);
//                 current = joint.parent;
//                 found = true;
//                 break;
//             }
//         }
//         if (!found)
//             throw std::runtime_error("No joint connects to " + current + ", can't reach base " + base);
//     }
//     chain.push_back(base);
//     reverse(chain.begin(), chain.end());
//     return chain;
// }

std::string MotionPlanners::generateSRDF(const std::string &robotName, const std::string &groupName, const std::string &baseLink, const std::string &tipLink,
                                         const std::vector<std::string> &robotLinks, const std::set<std::string> &envLinks, const std::vector<Joint> &joints)
{
    tinyxml2::XMLDocument doc;

    // Add XML declaration
    tinyxml2::XMLDeclaration *decl = doc.NewDeclaration("xml version=\"1.0\" ?");
    doc.InsertFirstChild(decl);

    tinyxml2::XMLNode *root = doc.NewElement("robot");
    doc.InsertEndChild(root);
    ((tinyxml2::XMLElement *)root)->SetAttribute("name", robotName.c_str());

    // Only use the kinematic chain from baseLink to tipLink
    std::map<std::string, Joint> childToJoint;
    for (const auto &joint : joints)
    {
        childToJoint[joint.child] = joint;
    }

    std::set<std::string> chainLinks;
    std::map<std::string, std::vector<std::string>> adjacencyList;
    std::set<std::pair<std::string, std::string>> parentChildPairs;

    std::string current = tipLink;
    while (current != baseLink && childToJoint.count(current))
    {
        const Joint &joint = childToJoint[current];
        chainLinks.insert(current);
        parentChildPairs.insert({joint.parent, joint.child});
        adjacencyList[joint.parent].push_back(joint.child);
        current = joint.parent;
    }
    chainLinks.insert(baseLink);

    // Add group and chain definition
    tinyxml2::XMLElement *group = doc.NewElement("group");
    group->SetAttribute("name", groupName.c_str());

    tinyxml2::XMLElement *chain = doc.NewElement("chain");
    chain->SetAttribute("base_link", baseLink.c_str());
    chain->SetAttribute("tip_link", tipLink.c_str());
    group->InsertEndChild(chain);
    root->InsertEndChild(group);

    // End effector
    tinyxml2::XMLElement *ee = doc.NewElement("end_effector");
    ee->SetAttribute("name", (groupName + "_ee").c_str());
    ee->SetAttribute("parent_link", tipLink.c_str());
    ee->SetAttribute("group", groupName.c_str());
    root->InsertEndChild(ee);

    // Virtual joint
    tinyxml2::XMLElement *vj = doc.NewElement("virtual_joint");
    vj->SetAttribute("name", "base");
    vj->SetAttribute("type", "fixed");
    vj->SetAttribute("parent_frame", "base");
    vj->SetAttribute("child_link", baseLink.c_str());
    root->InsertEndChild(vj);

    // Expand parent-child transitive closure in chain
    std::function<void(const std::string &, const std::string &, std::set<std::string> &)> findDescendants =
        [&](const std::string &root, const std::string &current, std::set<std::string> &visited)
    {
        if (visited.count(current))
            return;
        visited.insert(current);
        if (current != root)
        {
            parentChildPairs.insert({root, current});
        }
        if (adjacencyList.count(current))
        {
            for (const auto &child : adjacencyList[current])
            {
                findDescendants(root, child, visited);
            }
        }
    };

    for (const auto &[parent, children] : adjacencyList)
    {
        std::set<std::string> visited;
        findDescendants(parent, parent, visited);
    }

    // Collision matrix
    tinyxml2::XMLElement *collisionMatrix = doc.NewElement("collision_matrix");
    collisionMatrix->SetAttribute("default", "enabled");

    for (const auto &rl : robotLinks)
    {
        if (!chainLinks.count(rl))
            continue;
        if( rl == robotLinks[0])  // WORKAROUND: Ignore collisions of first Link with environment
            continue;

        for (const auto &el : envLinks)
        {
            tinyxml2::XMLElement *pair = doc.NewElement("pair");
            pair->SetAttribute("link1", rl.c_str());
            pair->SetAttribute("link2", el.c_str());
            collisionMatrix->InsertEndChild(pair);
        }
    }

    for (size_t i = 0; i < robotLinks.size(); ++i)
    {
        const auto &l1 = robotLinks[i];
        if (!chainLinks.count(l1))
            continue;

        for (size_t j = i + 1; j < robotLinks.size(); ++j)
        {
            const auto &l2 = robotLinks[j];
            if (!chainLinks.count(l2))
                continue;

            if (parentChildPairs.count({l1, l2}) == 0 && parentChildPairs.count({l2, l1}) == 0)
            {
                tinyxml2::XMLElement *pair = doc.NewElement("pair");
                pair->SetAttribute("link1", l1.c_str());
                pair->SetAttribute("link2", l2.c_str());
                collisionMatrix->InsertEndChild(pair);
            }
        }
    }

    root->InsertEndChild(collisionMatrix);

    // Output SRDF as string
    tinyxml2::XMLPrinter printer(nullptr, false, 0);
    doc.Print(&printer);
    return printer.CStr();
}

void MotionPlanners::parseURDF(const std::string &path, std::set<std::string> &links, std::vector<Joint> &joints, std::string &robotName)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        throw std::runtime_error("Failed to load URDF file");
    }

    tinyxml2::XMLElement *root = doc.RootElement();
    if (!root || std::string(root->Name()) != "robot")
    {
        throw std::runtime_error("Invalid URDF: root element is not <robot>");
    }

    const char *nameAttr = root->Attribute("name");
    if (!nameAttr)
    {
        throw std::runtime_error("URDF <robot> element missing 'name' attribute");
    }
    robotName = std::string(nameAttr);

    for (tinyxml2::XMLElement *link = root->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
    {
        links.insert(link->Attribute("name"));
    }

    for (tinyxml2::XMLElement *joint = root->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint"))
    {
        Joint j;
        j.name = joint->Attribute("name");
        j.type = joint->Attribute("type");
        j.parent = joint->FirstChildElement("parent")->Attribute("link");
        j.child = joint->FirstChildElement("child")->Attribute("link");
        joints.push_back(j);
    }
}

std::vector<std::string> MotionPlanners::getLinksFromChain(const std::vector<Joint> &joints, const std::string &startLink, const std::string &endLink)
{
    std::map<std::string, Joint> childToJoint;
    for (const auto &joint : joints)
    {
        childToJoint[joint.child] = joint;
    }

    std::vector<std::string> chain;
    std::string current = endLink;
    while (current != startLink && childToJoint.count(current))
    {
        chain.push_back(current);
        current = childToJoint[current].parent;
    }
    chain.push_back(startLink);
    std::reverse(chain.begin(), chain.end());
    return chain;
}

void MotionPlanners::generateSRDFFiles(const std::string &urdfPath, const std::string &manipName,
                                       const std::string &base, const std::string &tip,
                                       const std::string &firstLink, const std::string &outputFile)
{
    std::set<std::string> links;
    std::vector<Joint> joints;
    std::string urdf_robot_name;
    parseURDF(urdfPath, links, joints, urdf_robot_name);

    std::vector<std::string> chain = getLinksFromChain(joints, firstLink, tip);
    std::set<std::string> chainSet(chain.begin(), chain.end());

    std::set<std::string> envLinks;
    for (const auto &link : links)
    {
        if (chainSet.find(link) == chainSet.end())
        {
            envLinks.insert(link);
        }
    }

    std::string srdf = generateSRDF(urdf_robot_name, manipName, base, tip, chain, envLinks, joints);

    std::ofstream out(outputFile);
    out << srdf;
    out.close();

    std::cout << "SRDF file generated: " << outputFile << std::endl;
}

/**
 *
 */
void MotionPlanners::printCollisionObjectNames()
{
    std::cout << "Links in Collision" << std::endl;
    for (const auto &[link1, link2] : this->collision_object_names_)
    {
        std::cout << "Link 1 " << link1 << std::endl;
        std::cout << "Link 2 " << link2 << std::endl;
    }
}

/**
 *
 */
bool MotionPlanners::checkNaN(base::samples::Joints joint_value)
{
    for (const auto &element : joint_value.elements)
    {
        if (std::isnan(element.position))
        {
            return false;
        }
    }
    return true;
}

/**
 *
 */
bool MotionPlanners::removeObject(const std::string &obj_name, const std::string &attach_link)
{
    // First remove the object from the env.
    motion_planners::ModelObject remove_object;
    remove_object.object_name = obj_name;
    remove_object.operation = collision_detection::REMOVE;
    remove_object.model_type = collision_detection::MESH;
    if (attach_link == this->robot_links.kuka.ee || attach_link == this->robot_links.vispa.ee ||
        attach_link == "VISPA_SI_0_link" || attach_link == "IIWA14_SI_0_link")
    {
        return handleGraspObject(remove_object);
    }
    else
    {
        return handleCollisionObjectInWorld(remove_object);
    }
}

/**
 *
 */
bool MotionPlanners::addObject(const std::string &obj_name, const std::string &attach_link, const base::Pose &obj_rel_pose)
{
    // First remove the object
    bool remove_res = removeObject(obj_name, attach_link);

    // Add the object as grasp obj in the environment
    motion_planners::ModelObject add_grasp_object;
    add_grasp_object.object_name = obj_name;
    add_grasp_object.operation = collision_detection::ADD;
    add_grasp_object.model_type = collision_detection::MESH;
    std::string urdf_path = robot_model_->getURDFfileAbsolutePath();
    add_grasp_object.object_path = getCollisionMeshAbsolutePath(urdf_path, obj_name);
    add_grasp_object.attach_link_name = attach_link;
    add_grasp_object.relative_pose = obj_rel_pose;
    if (attach_link == this->robot_links.kuka.ee || attach_link == this->robot_links.vispa.ee ||
        attach_link == "VISPA_SI_0_link" || attach_link == "IIWA14_SI_0_link")
    {
        return handleGraspObject(add_grasp_object);
    }
    else
    {
        return handleCollisionObjectInWorld(add_grasp_object);
    }
}

/**
 *
 */
std::string MotionPlanners::getCollisionMeshAbsolutePath(const std::string &urdf_path, const std::string &obj_name)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        throw std::runtime_error("Failed to load URDF file: " + urdf_path);
    }

    tinyxml2::XMLElement *root = doc.RootElement();
    if (!root || std::string(root->Name()) != "robot")
    {
        throw std::runtime_error("Invalid URDF: root element is not <robot>");
    }

    for (tinyxml2::XMLElement *link = root->FirstChildElement("link"); link; link = link->NextSiblingElement("link"))
    {
        const char *link_name = link->Attribute("name");
        if (!link_name || obj_name != link_name)
        {
            continue;
        }

        tinyxml2::XMLElement *collision = link->FirstChildElement("collision");
        if (!collision)
            continue;

        tinyxml2::XMLElement *geometry = collision->FirstChildElement("geometry");
        if (!geometry)
            continue;

        tinyxml2::XMLElement *mesh = geometry->FirstChildElement("mesh");
        if (!mesh)
            continue;

        const char *mesh_file = mesh->Attribute("filename");
        if (!mesh_file)
            continue;

        boost::filesystem::path mesh_path(mesh_file);
        if (mesh_path.is_relative())
        {
            boost::filesystem::path base_dir = boost::filesystem::path(urdf_path).parent_path();
            mesh_path = base_dir / mesh_path;
        }

        return boost::filesystem::absolute(mesh_path).string();
    }

    throw std::runtime_error("No collision mesh found for link: " + obj_name);
}
