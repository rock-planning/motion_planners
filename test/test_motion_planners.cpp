#include <motion_planners/MotionPlanners.hpp>

using namespace motion_planners;


motion_planners::EnvironmentConfig getCollisionDetectionConfig()
{
    motion_planners::EnvironmentConfig config;
    config.env_frame = "base_link";
    config.collision_detection_config.collision_library = collision_detection::FCL;
    config.collision_detection_config.collision_info_type = collision_detection::MULTI_CONTACT;
    config.collision_detection_config.stop_after_first_collision = true;
    config.collision_detection_config.calculate_distance_information = false;
    config.collision_detection_config.max_num_collision_contacts = 1;
    config.collision_detection_config.env_debug_config.save_octree = false;
    config.collision_detection_config.env_debug_config.save_octree_filename="";
    config.collision_detection_config.env_debug_config.save_octree_path="";
    config.env_object_name = "environment";
    collision_detection::CollisionLinkName disabled_collision("environment", "base_link");
    config.disabled_collision_pair.collision_link_names.push_back(disabled_collision);
    return config;
}

kinematics_library::KinematicsConfig getKinematicsConfig(std::string test_folder_path)
{
    kinematics_library::KinematicsConfig config;

    config.config_name = "kuka_arm";
    config.base_name = "base_link";
    config.tip_name = "link_7";
    config.urdf_file = test_folder_path +"./data/kuka_iiwa.urdf";
    config.kinematic_solver = kinematics_library::KDL;
    config.solver_config_abs_path = test_folder_path +"./config";
    config.solver_config_filename = "kdl_config.yml";

    return config;
}

robot_model::RobotModelConfig getRobotModelConfig(std::string test_folder_path)
{
    robot_model::RobotModelConfig config;
    // srdf file abs path
    config.srdf_file = test_folder_path +"./data/kuka_iiwa.srdf"; 
    // urdf file abs path
    config.urdf_file = test_folder_path +"./data/kuka_iiwa.urdf";
    // planning group
    config.planning_group_name = "manipulator";

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
    config.planner_config.planner_specific_config = test_folder_path +"./config/ompl.yml"; //stompl.yml
    // planner
    config.planner_config.planner = motion_planners::OMPL; //motion_planners::STOMP;
    
    // get collision detection config
    config.env_config = getCollisionDetectionConfig();
    return config;
}

base::samples::Joints convertToBaseJoints(const std::vector<double> &data)
{
    base::samples::Joints joint_values;
    joint_values.names = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"};
    joint_values.elements.resize(7);
    assert(joint_values.size() == data.size());
    for(size_t i = 0; i < data.size(); i++)
        joint_values.elements[i].position =  data[i];
    
    return joint_values;
}

void printTrajectory(const base::JointsTrajectory &traj)
{
    std::cout<<"Number of timestep :"<<traj.getTimeSteps()<<". Number of joints = "<<traj.getNumberOfJoints()<<std::endl;
    for(size_t i = 0; i < traj.getTimeSteps(); i++)    
    {
        for(size_t j = 0; j < traj.elements.size(); j++)    
        {
            std::cout<<traj.elements[j][i].position<<"  ";
        }
        std::cout<<std::endl;
    }
}

void printPlannerStatus(motion_planners::PlannerStatus &planner_status)
{
    switch(planner_status.statuscode)
    {
        case motion_planners::PlannerStatus::PATH_FOUND:
            std::cout<<"PATH_FOUND"<<std::endl; break;
        case motion_planners::PlannerStatus::NO_PATH_FOUND:
            std::cout<<"NO_PATH_FOUND"<<std::endl; break;
        case motion_planners::PlannerStatus::START_STATE_IN_COLLISION:
            std::cout<<"START_STATE_IN_COLLIfor(size_t i = 0; i < traj.elements.size(); i++)SION"<<std::endl; break;
        case motion_planners::PlannerStatus::GOAL_STATE_IN_COLLISION:
            std::cout<<"GOAL_STATE_IN_COLLISION"<<std::endl; break;
        case motion_planners::PlannerStatus::START_JOINTANGLES_NOT_AVAILABLE:
            std::cout<<"START_JOINTANGLES_NOT_AVAILABLE"<<std::endl; break;
        case motion_planners::PlannerStatus::GOAL_JOINTANGLES_NOT_AVAILABLE:
            std::cout<<"GOAL_JOINTANGLES_NOT_AVAILABLE"<<std::endl; break;
        case motion_planners::PlannerStatus::PLANNING_REQUEST_SUCCESS:
            std::cout<<"PLANNING_REQUEST_SUCCESS"<<std::endl; break;
        case motion_planners::PlannerStatus::CONSTRAINED_POSE_NOT_WITHIN_BOUNDS:
            std::cout<<"CONSTRAINED_POSE_NOT_WITHIN_BOUNDS"<<std::endl; break;
        case motion_planners::PlannerStatus::TIMEOUT:
            std::cout<<"TIMEOUT"<<std::endl; break;
        case motion_planners::PlannerStatus::INVALID_START_STATE:
            std::cout<<"INVALID_START_STATE"<<std::endl; break;
        case motion_planners::PlannerStatus::INVALID_GOAL_STATE:
            std::cout<<"INVALID_GOAL_STATE"<<std::endl; break;
        case motion_planners::PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
            std::cout<<"UNRECOGNIZED_GOAL_TYPE"<<std::endl; break;
        case motion_planners::PlannerStatus::APPROXIMATE_SOLUTION:
            std::cout<<"APPROXIMATE_SOLUTION"<<std::endl; break;
        case motion_planners::PlannerStatus::EXACT_SOLUTION:
            std::cout<<"PATH_FOUND"<<std::endl; break;
        case motion_planners::PlannerStatus::ROBOTMODEL_INITIALISATION_FAILED:
            std::cout<<"ROBOTMODEL_INITIALISATION_FAILED"<<std::endl; break;
        case motion_planners::PlannerStatus::PLANNER_INITIALISATION_FAILED:
            std::cout<<"PLANNER_INITIALISATION_FAILED"<<std::endl; break;
        case motion_planners::PlannerStatus::CRASH:
            std::cout<<"CRASH"<<std::endl; break;
        case motion_planners::PlannerStatus::KINEMATIC_ERROR:
        {
            switch(planner_status.kinematic_status.statuscode)
            {
                case kinematics_library::KinematicsStatus::KDL_TREE_FAILED:
                    std::cout<<"KDL_TREE_FAILED"<<std::endl; break;
                case kinematics_library::KinematicsStatus::KDL_CHAIN_FAILED:
                    std::cout<<"KDL_CHAIN_FAILED"<<std::endl; break;
                case kinematics_library::KinematicsStatus::URDF_FAILED:
                    std::cout<<"URDF_FAILED"<<std::endl; break;
                case kinematics_library::KinematicsStatus::NO_KINEMATIC_SOLVER_FOUND:
                    std::cout<<"NO_KINEMATIC_SOLVER_FOUND"<<std::endl; break;
                case kinematics_library::KinematicsStatus::IK_FOUND:
                    std::cout<<"IK_FOUND"<<std::endl; break;
                case kinematics_library::KinematicsStatus::NO_IK_SOLUTION:
                    std::cout<<"NO_IK_SOLUTION"<<std::endl; break;
                case kinematics_library::KinematicsStatus::NO_FK_SOLUTION:
                    std::cout<<"NO_FK_SOLUTION"<<std::endl; break; 	    
                case kinematics_library::KinematicsStatus::IK_TIMEOUT:
                    std::cout<<"IK_TIMEOUT"<<std::endl; break;        
                case kinematics_library::KinematicsStatus::IK_JOINTLIMITS_VIOLATED:
                    std::cout<<"IK_JOINTLIMITS_VIOLATED"<<std::endl; break;
               case kinematics_library::KinematicsStatus::NO_CONFIG_FILE:
                    std::cout<<"NO_KINEMATIC_CONFIG_FILE"<<std::endl; break;
                case kinematics_library::KinematicsStatus::CONFIG_READ_ERROR:
                    std::cout<<"KINEMATIC_CONFIG_READ_ERROR"<<std::endl; break;
                case kinematics_library::KinematicsStatus::INVALID_STATE:
                    std::cout<<"INVALID_KINEMATIC_STATE"<<std::endl; break;
                case kinematics_library::KinematicsStatus::APPROX_IK_SOLUTION:
                    std::cout<<"IK_FOUND"<<std::endl; break;
                default:
                {
                    std::cout<<"unknown Kinematics state"<<planner_status.kinematic_status.statuscode<<std::endl;                    
                    throw new std::runtime_error("This kinematic status is unknown");		    
                    break;
                }
            }
            break;
        }
        case motion_planners::PlannerStatus::INVALID:
            std::cout<<"UNKNOWN_STATE"<<std::endl; break;
        default:
        {
            LOG_ERROR("[PlannerTask]: Planner is in an unknown state. The current state value is %d", planner_status.statuscode);
            std::cout<<"UNKNOWN_STATE"<<std::endl;          
            break;
        }
    }
}

int main(int argc, char * argv[])
{
    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
    std::cout<<"!             Test function fot testing motion planners          !\n";
    std::cout<<"!./test_motion_planners absolute_path_to_test_folder             !\n";
    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n";
    if(argc != 2)
    {
        std::cout<<"The test function expect the absoulte path to the test folder"<<std::endl;
        return 0;
    }

    std::string test_folder_path = argv[1];
    std::cout<<"The given absolute path: "<<test_folder_path.c_str()<<std::endl;

    // get planner config
    motion_planners::Config config = getMotionPlannerConfig(test_folder_path);

    // create motion planner object
    motion_planners::MotionPlanners planner(config);
    // initialise the planner
    PlannerStatus planner_status;
    if(!planner.initialize(planner_status))
    {
        std::cout<<"Motion planner failed at initialization. Refer to planner status to get the error information"<<std::endl;
        printPlannerStatus(planner_status);
    }

    // assign planning request
    std::vector<double> start_vec_values = {0.5, 0.5, 0.5, -1.5, 0.5, 0.5, 0.5};    
    base::samples::Joints start_joint_values = convertToBaseJoints(start_vec_values);
    std::vector<double> target_vec_values = {-1.5, -1.5, -1.5, 1.5, -1.5, -1.5, -0.5};
    base::samples::Joints target_joint_values = convertToBaseJoints(target_vec_values);
    
    if(planner.assignPlanningRequest(start_joint_values, target_joint_values, planner_status))
    {
        // plan only if the planning request is success
        planner.setStartAndGoal();  // this function will initialise the start and goal for the planner
        double solving_time = 0.0;
        base::JointsTrajectory solution;
        if(planner.solve(solution, planner_status, solving_time))
        {
            std::cout<<"Path Found"<<std::endl;
            printTrajectory(solution);
        }
        else
        {
            std::cout<<"No Path Found. Refer to planner status to get the error information"<<std::endl;
            printPlannerStatus(planner_status);
        }        
    }
    else
    {
        std::cout<<"Assigning planning request failed. Refer to planner status to get the error information"<<std::endl;
        printPlannerStatus(planner_status);
    }        
    return 0;
}