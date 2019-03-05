#include <vector>
#include <string>

#include <MotionPlanners.hpp>


using namespace motion_planners;


MotionPlanners::MotionPlanners(Config config)
{
    config_ = config;
}

MotionPlanners::~MotionPlanners()
{}


bool MotionPlanners::initialize(PlannerStatus &planner_status)
{
    // CAUTION: Don't use different collision library for robot and world.
    // IN FCL wrapper the base pointer is downcasted.
    collision_detection::AbstractCollisionPtr robot_collision_detector = collision_factory_.getCollisionDetector(collision_detection::FCL, config_.env_config.octree_debug_config);
    collision_detection::AbstractCollisionPtr world_collision_detector = collision_factory_.getCollisionDetector(collision_detection::FCL, config_.env_config.octree_debug_config);
    // get the kinematics solver
    kinematics_library::AbstractKinematicPtr robot_kinematics =  kinematics_factory_.getKinematicsSolver(config_.planner_config.kinematics_config, 
                                                                                                         planner_status.kinematic_status);
    if(robot_kinematics==NULL)
        return false;


    // initialise robot model    
    robot_model_.reset(new RobotModel(config_.planner_config.robot_model_config.urdf_file, config_.planner_config.robot_model_config.srdf_file, 
    config_.planner_config.robot_model_config.planning_group_name));
    robot_model_->setRobotCollisionDetector(robot_collision_detector);
    robot_model_->setWorldCollisionDetector(world_collision_detector); 
    robot_model_->setKinematicsSolver(robot_kinematics);
    if(!robot_model_->initialization())
    {
        planner_status.statuscode = PlannerStatus::ROBOTMODEL_INITIALISATION_FAILED;
        return false;
    }

    robot_model_->setDisabledEnvironmentCollision(config_.env_config.disabled_collision_pair);

    PlannerFactory planner_factory;

    planner_ = planner_factory.getPlannerTask(config_.planner_config.planner);

    if(!planner_->initializePlanner(robot_model_, config_.planner_config.planner_specific_config))
    {
        planner_status.statuscode = PlannerStatus::PLANNER_INITIALISATION_FAILED;
        return false;
    }

    std::string base_link, tip_link;
    planning_group_joints_.clear();
    robot_model_->getPlanningGroupJointinformation(config_.planner_config.robot_model_config.planning_group_name, 
                                                   planning_group_joints_, base_link, tip_link);

    goal_pose_.position = Eigen::Vector3d::Zero();
    goal_pose_.orientation = Eigen::Quaterniond::Identity();

    return true;
}

bool MotionPlanners::checkStartState(const base::samples::Joints &current_robot_status, PlannerStatus &planner_status )
{
    // check whether the start state is in collision

    robot_model_->updateJointGroup(current_robot_status);

    if(!robot_model_->isStateValid())
    {
        planner_status.statuscode = PlannerStatus::START_STATE_IN_COLLISION;
        collision_object_names_ = robot_model_->getCollisionObjectNames();
        return false;
    }
    else
    {
        // assign the start joint values from current robot status	  
        initial_joint_status_.clear();
        initial_joint_status_.resize(planning_group_joints_.size());

        for(size_t i = 0; i < planning_group_joints_.size(); i++)
        {
            try
            {
                base::JointState current_jointstate = current_robot_status.getElementByName(planning_group_joints_.at(i).first);		

                initial_joint_status_.names.at(i) = planning_group_joints_.at(i).first;
                initial_joint_status_.elements.at(i) = current_jointstate;		
            }
            catch(base::samples::Joints::InvalidName e) //Only catch exception to write more explicit error msgs
            {
                LOG_ERROR("[MotionPlanners]: Joint %s is given in planning group but is not available in the given start value", 
                          planning_group_joints_.at(i).first.c_str());
                return false;
            }
        }
    }
    return true;
}

bool MotionPlanners::checkGoalState(const base::samples::Joints &goal, PlannerStatus &planner_status )
{
    // check whether the goal state is in collision        
    robot_model_->updateJointGroup(goal);
    if(!robot_model_->isStateValid())
    {
        planner_status.statuscode = PlannerStatus::GOAL_STATE_IN_COLLISION;	    
        collision_object_names_ = robot_model_->getCollisionObjectNames();
    }
    else
    {
        planner_status.statuscode = PlannerStatus::PLANNING_REQUEST_SUCCESS;
        return true;
    }

    return false;
}

void MotionPlanners::updateOctomap(const std::shared_ptr<octomap::OcTree> &octomap)
{
    robot_model_->updateOctomap(octomap, config_.env_config.env_object_name);
    robot_model_->saveOctree();
}

void MotionPlanners::assignOctomapPlanningScene(const std::shared_ptr<octomap::OcTree> &octomap)
{
    //assign  a empty planning scene;
    robot_model_->assignPlanningScene(octomap, config_.env_config.env_frame, config_.env_config.env_object_name);
}

bool MotionPlanners::usePredictedTrajectory( base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    base::samples::Joints start, goal;
    solution.getJointsAtTimeStep(0, start);
    solution.getJointsAtTimeStep(solution.size(), goal);
    if(!assignPlanningRequest( start, goal, planner_status))
        return false;
    
    planner_->updateInitialTrajectory(solution);
}

bool MotionPlanners::assignPlanningRequest( const base::samples::Joints &start_jointvalues, const base::samples::Joints &target_jointvalues,
                                            PlannerStatus &planner_status)
{

    if (checkStartState(start_jointvalues, planner_status))
    {
        // assign the goal joint values from the target joint status
        goal_joint_status_.clear();
        goal_joint_status_.resize(planning_group_joints_.size());
        for(size_t i = 0; i < planning_group_joints_.size(); i++)
        {
            try
            {
                goal_joint_status_.names.at(i)		= planning_group_joints_.at(i).first;
                goal_joint_status_.elements.at(i)	= target_jointvalues.getElementByName(planning_group_joints_.at(i).first);
            }
            catch(base::samples::Joints::InvalidName e) //Only catch exception to write more explicit error msgs
            {
                LOG_ERROR("[MotionPlanners]: Joint %s is given in planning group but is not available in the given target value", 
                          planning_group_joints_.at(i).first.c_str());
                return false;
            }
        }

        if(checkGoalState(goal_joint_status_, planner_status))
            return true;
    }
    return false;
}


bool MotionPlanners::assignPlanningRequest( const base::samples::Joints &start_jointvalues, const base::samples::RigidBodyState &target_pose,
                                            PlannerStatus &planner_status)
{

    if (checkStartState(start_jointvalues, planner_status))
    {
        // assign the goal joint values from the target joint status
        goal_pose_ = target_pose;
        // assign the goal joint values from the target joint status
        goal_joint_status_.clear();
        goal_joint_status_.resize ( planning_group_joints_.size() );

        robot_model_->robot_kinematics_->solveIK ( goal_pose_, start_jointvalues, ik_solution_, planner_status.kinematic_status );

        if ( planner_status.kinematic_status.statuscode == kinematics_library::KinematicsStatus::IK_FOUND ) 
        {

            for(std::vector<base::commands::Joints>::iterator it = ik_solution_.begin(); it != ik_solution_.end(); ++it)
            {
                for ( size_t i = 0; i < planning_group_joints_.size(); i++ )
                {
                    try
                    {
                        goal_joint_status_.names.at ( i )	= planning_group_joints_.at ( i ).first;
                        goal_joint_status_.elements.at ( i )    = it->getElementByName ( planning_group_joints_.at ( i ).first );
                    }
                    catch ( base::samples::Joints::InvalidName e )
                    {   //Only catch exception to write more explicit error msgs
                        LOG_ERROR ( "[MotionPlanners]: Joint %s is given in planning group but is not available for the goal value", 
                                    planning_group_joints_.at ( i ).first.c_str() );
                        return false;
                    }
                }

                if ( checkGoalState ( goal_joint_status_, planner_status ) )            
                    return true;            
            }
        }
    }
    return false;
}


bool MotionPlanners::convertModelObjectToURDFCollision(const motion_planners::ModelObject &known_object, std::shared_ptr<urdf::Collision> collision_object)
{
    // assign the object name
    collision_object->name = known_object.object_name;

    // assign the pose value for the grasp object
    collision_object->origin.position.x = known_object.relative_pose.position(0);
    collision_object->origin.position.y = known_object.relative_pose.position(1);
    collision_object->origin.position.z = known_object.relative_pose.position(2);    
    collision_object->origin.rotation.setFromQuaternion(known_object.relative_pose.orientation.x(),
                                                        known_object.relative_pose.orientation.y(),
                                                        known_object.relative_pose.orientation.z(),
                                                        known_object.relative_pose.orientation.w());

    if ( known_object.model_type == collision_detection::PRIMITIVES )
    {
        // box, cylinder or sphere
        if ( known_object.primitive_object.primitive_type == collision_detection::BOX )
        {
            std::shared_ptr<urdf::Box> urdf_box_ptr ( new urdf::Box );
            urdf_box_ptr->dim.x = known_object.primitive_object.dimensions.x();
            urdf_box_ptr->dim.y = known_object.primitive_object.dimensions.y();
            urdf_box_ptr->dim.z = known_object.primitive_object.dimensions.z();

            collision_object->geometry = urdf_box_ptr;
        }
        else if ( known_object.primitive_object.primitive_type == collision_detection::CYLINDER )
        {
            std::shared_ptr<urdf::Cylinder> urdf_cylinder_ptr ( new urdf::Cylinder );
            urdf_cylinder_ptr->radius = known_object.primitive_object.radius;
            urdf_cylinder_ptr->length = known_object.primitive_object.height;

            collision_object->geometry = urdf_cylinder_ptr;
        }
        else if ( known_object.primitive_object.primitive_type == collision_detection::SPHERE )
        {
            std::shared_ptr<urdf::Sphere> urdf_sphere_ptr ( new urdf::Sphere );
            urdf_sphere_ptr->radius = known_object.primitive_object.radius;

            collision_object->geometry = urdf_sphere_ptr;
        }
        else
        {
            LOG_INFO ( "[MotionPlanners]: Primitive object type is undefined" );
            return false;
        }
    }
    else if ( known_object.model_type == collision_detection::MESH )
    {
        std::shared_ptr<urdf::Mesh> urdf_mesh_ptr ( new urdf::Mesh );
        urdf_mesh_ptr->filename = known_object.object_path;

        collision_object->geometry = urdf_mesh_ptr;
    }
    else if (known_object.model_type == collision_detection::OCTREE)
    {}
    else
    {
        LOG_WARN ( "[MotionPlanners]: Object type is undefined" );
        return false;
    }
    return true;
}

bool MotionPlanners::handleCollisionObjectInWorld ( const motion_planners::ModelObject &known_object )
{
    if ( known_object.operation == collision_detection::RESET )
    {
        LOG_INFO ( "[MotionPlanners]: Received known object with RESET" );
        return false;
    }

    if( known_object.model_type == collision_detection::UNDEFINED)
    {
        LOG_INFO ( "[MotionPlanners]: Remove known object with name %s is of UNDEFINED type", known_object.object_name.c_str() );
        return false;
    }

    std::shared_ptr<urdf::Collision> collision_object = std::make_shared<urdf::Collision>();

    if ( convertModelObjectToURDFCollision ( known_object, collision_object ) )
    {
        if ( known_object.operation == collision_detection::REMOVE )
        {
            LOG_INFO ( "[MotionPlanners]: Remove known object with name %s", known_object.object_name.c_str() );

            if( (known_object.model_type == collision_detection::OCTREE))
                return robot_model_->removeObjectFromOctree(known_object.relative_pose.position, known_object.primitive_object.dimensions);
            
            if(!robot_model_->removeWorldObject ( known_object.object_name ))
                    return false;
            
        }
        else if ( known_object.operation == collision_detection::ADD )
        {
            LOG_INFO ( "[MotionPlanners]: Add known object with name %s", known_object.object_name.c_str() );
            robot_model_->addCollisionsToWorld ( collision_object, known_object.attach_link_name );
        }
        else
        {
            LOG_INFO ( "[MotionPlanners]: Unknown collision::operation received " );
            return false;
        }
    }
    else
        return false;

    return true;
}

bool MotionPlanners::handleGraspObject ( const motion_planners::ModelObject &known_object )
{
    if ( known_object.operation == collision_detection::RESET )
    {
        LOG_INFO ( "[MotionPlanners]: Received grasp object with RESET" );
        return false;
    }

    std::shared_ptr<urdf::Collision> collision_object = std::make_shared<urdf::Collision>();

    if ( convertModelObjectToURDFCollision ( known_object, collision_object ) )
    {
        if ( known_object.operation == collision_detection::REMOVE )
        {
            LOG_INFO ( "[MotionPlanners]: Remove known object with name %s", known_object.object_name.c_str() );
            if(!robot_model_->removeGraspObject ( known_object.object_name ))
                return false;
        }
        else if ( known_object.operation == collision_detection::ADD )
        {
            LOG_INFO ( "[MotionPlanners]: Add known object with name %s", known_object.object_name.c_str() );
            robot_model_->addGraspObject ( collision_object, known_object.attach_link_name );
        }
        else
        {
            LOG_INFO ( "[MotionPlanners]: Unknown collision::operation received " );
            return false;
        }
    }
    else
        return false;

    return true;
}

void MotionPlanners::setStartAndGoal()
{
    planner_->setStartGoalTrajectory(initial_joint_status_, goal_joint_status_);    
}

bool MotionPlanners::solve ( base::JointsTrajectory &solution, PlannerStatus &planner_status, double &time_taken )
{

    auto start_time = std::chrono::high_resolution_clock::now();

    bool res = planner_->solve ( solution, planner_status );

    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - start_time;
    time_taken = elapsed.count();

    return res;

}
