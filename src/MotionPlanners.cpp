#include <motion_planners/MotionPlanners.hpp>

using namespace motion_planners;

/**
 *      Constructors
 */
MotionPlanners::MotionPlanners() = default;

MotionPlanners::MotionPlanners(Config config) : config_(config) {}

/**
 *      Destructor
 */
MotionPlanners::~MotionPlanners() = default;

/**
 *      Load the config
 */
void MotionPlanners::loadConfig(Config config)
{
    config_ = config;
}

/**
 *      Initialize the planner
 */
bool MotionPlanners::initialize(PlannerStatus &planner_status)
{
    // create robotmodel
    robot_model_ = std::make_unique<RobotModel>(config_.planner_config.robot_model_config);

    // CAUTION: Don't use different collision library for robot and world.
    // IN FCL wrapper the base pointer is downcasted.
    auto robot_collision_detector = collision_factory_.getCollisionDetector(config_.env_config.collision_detection_config);
    auto world_collision_detector = collision_factory_.getCollisionDetector(config_.env_config.collision_detection_config);

    // add the collision detector to the robot model
    robot_model_->setRobotCollisionDetector(robot_collision_detector);
    robot_model_->setWorldCollisionDetector(world_collision_detector);

    // get the kinematics solver
    if (!assignKinematicsToRobotModel(config_.planner_config.kinematics_config, kin_solver_, planner_status))
        return false;

    // initialise robot model
    if (!robot_model_->initialization())
    {
        planner_status.statuscode = PlannerStatus::ROBOTMODEL_INITIALISATION_FAILED;
        return false;
    }

    // disable any collision with the environment
    robot_model_->setDisabledEnvironmentCollision(assignDisableCollisionObject(config_.env_config.disabled_collision_pair));

    // planner
    PlannerFactory planner_factory;
    planner_ = planner_factory.getPlannerTask(config_.planner_config.planner);
    if (!planner_->initializePlanner(robot_model_, config_.planner_config.planner_specific_config, this->num_waypoints))
    {
        planner_status.statuscode = PlannerStatus::PLANNER_INITIALISATION_FAILED;
        return false;
    }

    planning_group_joints_.clear();
    robot_model_->getPlanningGroupJointInformation(config_.planner_config.robot_model_config.planning_group_name,
                                                   planning_group_joints_);

    createNamedGroupStates(robot_model_->getSRDF());
    goal_pose_.position = Eigen::Vector3d::Zero();
    goal_pose_.orientation = Eigen::Quaterniond::Identity();

    planner_status.statuscode = PlannerStatus::INIT;
    return true;
}

/**
 *      Reinitialize the planner
 */
bool MotionPlanners::reInitializePlanner(PlannerStatus &planner_status, const std::string &planner_name, bool &robot_updated)
{
    planner_status.statuscode = PlannerStatus::INIT;
    if (robot_updated)
    {
        auto robot_collision_detector = collision_factory_.getCollisionDetector(config_.env_config.collision_detection_config);
        auto world_collision_detector = collision_factory_.getCollisionDetector(config_.env_config.collision_detection_config);
        robot_model_->setRobotCollisionDetector(robot_collision_detector);
        robot_model_->setWorldCollisionDetector(world_collision_detector);

        // reinitialise robot model
        if (!robot_model_->reinitialization())
        {
            planner_status.statuscode = PlannerStatus::ROBOTMODEL_INITIALISATION_FAILED;
            return false;
        }

        // disable any collision with the environment
        robot_model_->setDisabledEnvironmentCollision(assignDisableCollisionObject(config_.env_config.disabled_collision_pair));

        // planner
        PlannerFactory planner_factory;
        planner_ = planner_factory.getPlannerTask(config_.planner_config.planner);
        if (!planner_->initializePlanner(robot_model_, config_.planner_config.planner_specific_config, this->num_waypoints))
        {
            planner_status.statuscode = PlannerStatus::PLANNER_INITIALISATION_FAILED;
            return false;
        }

        planning_group_joints_.clear();
        robot_model_->getPlanningGroupJointInformation(robot_model_->getPlanningGroupName(),
                                                       planning_group_joints_);

        createNamedGroupStates(robot_model_->getSRDF());
        goal_pose_.position = Eigen::Vector3d::Zero();
        goal_pose_.orientation = Eigen::Quaterniond::Identity();
    }

    if (planner_name == "stomp")
    {
        return planner_->reInitializePlanner();
    }

    return true;
}

/**
 *
 */
bool MotionPlanners::assignKinematicsToRobotModel(const kinematics_library::KinematicsConfig &kinematics_config,
                                                  kinematics_library::AbstractKinematicPtr &robot_kinematics,
                                                  PlannerStatus &planner_status)
{

    robot_kinematics = kinematics_factory_.getKinematicsSolver(kinematics_config, planner_status.kinematic_status);
    if (!robot_kinematics)
    {
        planner_status.statuscode = PlannerStatus::KINEMATIC_ERROR;
        return false;
    }

    // Add the kinematic solver to the robot model
    robot_model_->setKinematicsSolver(kinematics_config.config_name, robot_kinematics);
    return true;
}

/**
 *
 */
bool MotionPlanners::checkStartState(const base::samples::Joints &current_robot_status, PlannerStatus &planner_status)
{
    // Make sure incoming data doesn't have any NaN in it
    if (!checkNaN(current_robot_status))
    {
        planner_status.statuscode = PlannerStatus::START_JOINTANGLES_NOT_AVAILABLE;
        return false;
    }

    // Check whether the start state is in collision
    robot_model_->updateJointGroup(current_robot_status);

    double collision_cost = 0.0;
    if (!robot_model_->isStateValid(collision_cost))
    {
        planner_status.statuscode = PlannerStatus::START_STATE_IN_COLLISION;
        collision_object_names_ = robot_model_->getCollidedObjectsNames();
        for (size_t i = 0; i < planning_group_joints_.size(); i++)
        {
            const auto &joint_name = planning_group_joints_[i].first;
        }
        return false;
    }

    // Assign the start joint values from current robot status
    initial_joint_status_.clear();
    initial_joint_status_.resize(planning_group_joints_.size());

    for (size_t i = 0; i < planning_group_joints_.size(); i++)
    {
        try
        {
            const auto &joint_name = planning_group_joints_[i].first;
            initial_joint_status_.names[i] = joint_name;
            initial_joint_status_.elements[i] = current_robot_status.getElementByName(joint_name);
        }
        catch (base::samples::Joints::InvalidName const &e)
        {
            LOG_ERROR("[MotionPlanners]: Joint %s is given in planning group but is not available in the given start value",
                      planning_group_joints_.at(i).first.c_str());
            return false;
        }
    }

    return true;
}

/**
 *
 */
bool MotionPlanners::checkGoalState(const base::samples::Joints &goal, PlannerStatus &planner_status)
{
    // Make sure incoming data doesn't have any NaN in it
    if (!checkNaN(goal))
    {
        planner_status.statuscode = PlannerStatus::GOAL_JOINTANGLES_NOT_AVAILABLE;
        return false;
    }

    // Check whether the goal state is in collision
    robot_model_->updateJointGroup(goal);
    double collision_cost = 0.0;
    if (!robot_model_->isStateValid(collision_cost))
    {
        planner_status.statuscode = PlannerStatus::GOAL_STATE_IN_COLLISION;
        collision_object_names_ = robot_model_->getCollidedObjectsNames();
        return false;
    }

    planner_status.statuscode = PlannerStatus::PLANNING_REQUEST_SUCCESS;
    return true;
}

/**
 *
 */
void MotionPlanners::updateOctomap(const std::shared_ptr<octomap::OcTree> &octomap)
{
    robot_model_->updateOctomap(octomap, config_.env_config.env_object_name);
    if (config_.env_config.collision_detection_config.env_debug_config.save_octree)
    {
        robot_model_->saveOctree();
    }
}

/**
 *
 */
void MotionPlanners::assignOctomapPlanningScene(const std::shared_ptr<octomap::OcTree> &octomap)
{
    // assign an empty planning scene;
    robot_model_->assignPlanningScene(octomap, config_.env_config.env_frame, config_.env_config.env_object_name);
}

/**
 *
 */
bool MotionPlanners::usePredictedTrajectory(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    base::samples::Joints start, goal;
    solution.getJointsAtTimeStep(0, start);
    solution.getJointsAtTimeStep(solution.getTimeSteps() - 1, goal);

    if (!assignPlanningRequest(start, goal, planner_status))
    {
        return false;
    }

    planner_->updateInitialTrajectory(solution);
    return true;
}

/**
 * Planning Request for Joint Space Planning
 */
bool MotionPlanners::assignPlanningRequest(const base::samples::Joints &start_jointvalues,
                                           const base::samples::Joints &target_jointvalues,
                                           PlannerStatus &planner_status)
{
    planning_type_ = false;

    if (!checkStartState(start_jointvalues, planner_status))
    {
        return false;
    }

    // assign the goal joint values from the target joint status
    goal_joint_status_.clear();
    goal_joint_status_.resize(planning_group_joints_.size());

    for (size_t i = 0; i < planning_group_joints_.size(); i++)
    {
        try
        {
            goal_joint_status_.names.at(i) = planning_group_joints_.at(i).first;
            goal_joint_status_.elements.at(i) = target_jointvalues.getElementByName(planning_group_joints_.at(i).first);
        }
        catch (base::samples::Joints::InvalidName const &e) // Only catch exception to write more explicit error msgs
        {
            LOG_ERROR("[MotionPlanners]: Joint %s is given in planning group but is not available in the given target value",
                      planning_group_joints_.at(i).first.c_str());
            return false;
        }
    }

    if (checkGoalState(goal_joint_status_, planner_status))
    {
        constrainted_target_.use_constraint = motion_planners::NO_CONSTRAINT;
        return true;
    }

    return false;
}

/**
 * Planning Request for Cartesian Space Planning
 */
bool MotionPlanners::assignPlanningRequest(const base::samples::Joints &start_jointvalues,
                                           const base::samples::RigidBodyState &target_pose,
                                           PlannerStatus &planner_status)
{
    planning_type_ = true;
    ik_sol_numeral_ = 0;
    if (!checkStartState(start_jointvalues, planner_status))
    {
        return false;
    }

    // assign the goal joint values from the target joint status
    goal_pose_ = target_pose;
    // assign the goal joint values from the target joint status
    goal_joint_status_.clear();
    goal_joint_status_.resize(planning_group_joints_.size());

    kin_solver_->solveIK(goal_pose_, start_jointvalues, ik_solution_, planner_status.kinematic_status);

    ik_solution_.erase(
        std::remove_if(
            ik_solution_.begin(),
            ik_solution_.end(),
            [&](const base::commands::Joints &joints)
            {
                return !checkGoalState(joints, planner_status);
            }),
        ik_solution_.end());

    // printIKSolution(ik_solution_);

    if (planner_status.kinematic_status.statuscode != kinematics_library::KinematicsStatus::IK_FOUND &&
        planner_status.kinematic_status.statuscode != kinematics_library::KinematicsStatus::APPROX_IK_SOLUTION)
    {
        planner_status.statuscode = PlannerStatus::KINEMATIC_ERROR;
        return false;
    }

    // printPlanningGroupJoints(planning_group_joints_);
    for (const auto &solution : ik_solution_)
    {
        ik_sol_numeral_++;
        for (size_t i = 0; i < planning_group_joints_.size(); i++)
        {
            try
            {
                goal_joint_status_.names.at(i) = planning_group_joints_.at(i).first;
                goal_joint_status_.elements.at(i) = solution.getElementByName(planning_group_joints_.at(i).first);
            }
            catch (base::samples::Joints::InvalidName const &e)
            { // Only catch exception to write more explicit error msgs
                LOG_ERROR("[MotionPlanners]: Joint %s is given in planning group but is not available for the goal value",
                          planning_group_joints_.at(i).first.c_str());
                return false;
            }
        }

        if (checkGoalState(goal_joint_status_, planner_status))
        {
            constrainted_target_.use_constraint = motion_planners::NO_CONSTRAINT;
            return true;
        }
    }

    return false;
}

/**
 *
 */
bool MotionPlanners::assignPlanningRequest(const base::samples::Joints &start_jointvalues,
                                           const std::string &target_group_state,
                                           PlannerStatus &planner_status)
{
    planning_type_ = false;

    if (!checkStartState(start_jointvalues, planner_status))
    {
        return false;
    }

    // assign the goal joint values from the target group state
    auto it = named_group_states_.find(target_group_state);
    if (it == named_group_states_.end())
    {
        LOG_ERROR("[MotionPlanners]: Group State %s does not exist in the named group states for the planning group", target_group_state.c_str());
        return false;
    }

    auto joint_map = it->second;
    goal_joint_status_.clear();
    goal_joint_status_.resize(planning_group_joints_.size());

    for (size_t i = 0; i < planning_group_joints_.size(); i++)
    {
        try
        {
            goal_joint_status_.names.at(i) = planning_group_joints_.at(i).first;
            auto joint_it = joint_map.find((planning_group_joints_.at(i).first));
            goal_joint_status_.elements.at(i).position = joint_it->second;
        }
        catch (base::samples::Joints::InvalidName const &e) // Only catch exception to write more explicit error msgs
        {
            LOG_ERROR("[MotionPlanners]: Joint %s is given in planning group but is not available in the given target value",
                      planning_group_joints_.at(i).first.c_str());
            return false;
        }
    }
    for (size_t i = 0; i < goal_joint_status_.size(); ++i)
    {
        double diff = goal_joint_status_.elements[i].position - initial_joint_status_.elements[i].position;
        // set new goal positions so that are only rotations with a value below of PI
        if (diff > M_PI)
        {
            goal_joint_status_.elements.at(i).position -= 2 * M_PI;
        }
        else if (diff < -M_PI)
        {
            goal_joint_status_.elements.at(i).position += 2 * M_PI;
        }
        LOG_DEBUG("[MotionPlanners]: Named Goal Joint Value  for Joint %s = %f",
                  goal_joint_status_.names.at(i).c_str(),
                  goal_joint_status_.elements.at(i).position);
    }
    if (checkGoalState(goal_joint_status_, planner_status))
    {
        constrainted_target_.use_constraint = motion_planners::NO_CONSTRAINT;
        return true;
    }

    return false;
}

/**
 *
 */
bool MotionPlanners::assignPlanningRequest(const base::samples::Joints &start_jointvalues,
                                           const ConstraintPlanning &constrainted_target,
                                           PlannerStatus &planner_status)
{
    planning_type_ = false;
    bool result = false;

    switch (constrainted_target.use_constraint)
    {
    case motion_planners::JOINTS_CONSTRAINT:
        result = assignPlanningRequest(start_jointvalues, constrainted_target.target_joints_value, planner_status);
        break;

    case motion_planners::NO_CONSTRAINT:
        planner_status.statuscode = PlannerStatus::NO_CONSTRAINT_AVAILABLE;
        return false;

    default: // Handle pose constraints
        result = assignPlanningRequest(start_jointvalues, constrainted_target.target_pose, planner_status);
        break;
    }

    if (result)
    {
        constrainted_target_ = constrainted_target;
    }

    return result;
}

/**
 *
 */
bool MotionPlanners::convertModelObjectToURDFCollision(const motion_planners::ModelObject &known_object, std::shared_ptr<urdf::Collision> collision_object)
{
    // Assign the object name
    collision_object->name = known_object.object_name;

    // Assign the pose value for the object
    collision_object->origin.position.x = known_object.relative_pose.position(0);
    collision_object->origin.position.y = known_object.relative_pose.position(1);
    collision_object->origin.position.z = known_object.relative_pose.position(2);
    collision_object->origin.rotation.setFromQuaternion(
        known_object.relative_pose.orientation.x(),
        known_object.relative_pose.orientation.y(),
        known_object.relative_pose.orientation.z(),
        known_object.relative_pose.orientation.w());

    // Create the appropriate geometry based on model type
    switch (known_object.model_type)
    {
    case collision_detection::PRIMITIVES:
    {
        switch (known_object.primitive_object.primitive_type)
        {
        case collision_detection::BOX:
        {
            auto urdf_box = std::make_shared<urdf::Box>();
            urdf_box->dim.x = known_object.primitive_object.dimensions.x();
            urdf_box->dim.y = known_object.primitive_object.dimensions.y();
            urdf_box->dim.z = known_object.primitive_object.dimensions.z();
            collision_object->geometry = urdf_box;
            break;
        }

        case collision_detection::CYLINDER:
        {
            auto urdf_cylinder = std::make_shared<urdf::Cylinder>();
            urdf_cylinder->radius = known_object.primitive_object.radius;
            urdf_cylinder->length = known_object.primitive_object.height;
            collision_object->geometry = urdf_cylinder;
            break;
        }

        case collision_detection::SPHERE:
        {
            auto urdf_sphere = std::make_shared<urdf::Sphere>();
            urdf_sphere->radius = known_object.primitive_object.radius;
            collision_object->geometry = urdf_sphere;
            break;
        }

        default:
            LOG_INFO("[MotionPlanners]: Primitive object type is undefined");
            return false;
        }
        break;
    }

    case collision_detection::MESH:
    {
        auto urdf_mesh = std::make_shared<urdf::Mesh>();
        urdf_mesh->filename = known_object.object_path;
        collision_object->geometry = urdf_mesh;
        break;
    }

    case collision_detection::OCTREE:
        // Special handling for octree if needed
        break;

    default:
        LOG_WARN("[MotionPlanners]: Object type is undefined");
        return false;
    }

    return true;
}

/**
 *
 */
bool MotionPlanners::handleCollisionObjectInWorld(const motion_planners::ModelObject &known_object)
{
    if (known_object.operation == collision_detection::RESET)
    {
        LOG_INFO("[MotionPlanners]: Received known object with RESET");
        return false;
    }

    if (known_object.model_type == collision_detection::UNDEFINED)
    {
        LOG_INFO("[MotionPlanners]: Remove known object with name %s is of UNDEFINED type",
                 known_object.object_name.c_str());
        return false;
    }

    switch (known_object.operation)
    {
    case collision_detection::REMOVE:
    {
        LOG_INFO("[MotionPlanners]: Remove known object with name %s", known_object.object_name.c_str());

        if (known_object.model_type == collision_detection::OCTREE)
        {
            return robot_model_->removeObjectFromOctree(
                known_object.relative_pose.position,
                known_object.primitive_object.dimensions);
        }

        return robot_model_->removeWorldObject(known_object.object_name);
    }

    case collision_detection::ADD:
    {
        auto collision_object = std::make_shared<urdf::Collision>();
        if (!convertModelObjectToURDFCollision(known_object, collision_object))
        {
            return false;
        }
        LOG_INFO("[MotionPlanners]: Add known object with name %s", known_object.object_name.c_str());
        robot_model_->addCollisionsToWorld(collision_object, known_object.attach_link_name);
        return true;
    }

    default:
    {
        LOG_INFO("[MotionPlanners]: Unknown collision::operation received");
        return false;
    }
    }

    return false;
}

/**
 *
 */
bool MotionPlanners::handleGraspObject(const motion_planners::ModelObject &known_object)
{
    if (known_object.operation == collision_detection::RESET)
    {
        LOG_INFO("[MotionPlanners]: Received grasp object with RESET");
        return false;
    }

    switch (known_object.operation)
    {
    case collision_detection::REMOVE:
    {
        LOG_INFO("[MotionPlanners]: Remove known object with name %s", known_object.object_name.c_str());
        return robot_model_->removeGraspObject(known_object.object_name); // known_object.object_name is string
    }

    case collision_detection::ADD:
    {
        auto collision_object = std::make_shared<urdf::Collision>();

        if (!convertModelObjectToURDFCollision(known_object, collision_object))
        {
            return false;
        }
        LOG_INFO("[MotionPlanners]: Add known object with name %s", known_object.object_name.c_str());
        robot_model_->addGraspObject(collision_object, known_object.attach_link_name);
        return true;
    }

    default:
    {
        LOG_INFO("[MotionPlanners]: Unknown collision::operation received");
        return false;
    }
    }
    return false;
}

/**
 *
 */
void MotionPlanners::setStartAndGoal()
{
    planner_->setConstraints(constrainted_target_);
    planner_->setStartGoalTrajectory(initial_joint_status_, goal_joint_status_);
}

/**
 *
 */
bool MotionPlanners::solve(base::JointsTrajectory &solution, PlannerStatus &planner_status, double &time_taken)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    bool res = planner_->solve(solution, planner_status);

    // Try alternative IK solutions if needed for pose-based planning
    if (!(planner_status.statuscode == PlannerStatus::PATH_FOUND ||
          planner_status.statuscode == PlannerStatus::EXACT_SOLUTION ||
          planner_status.statuscode == PlannerStatus::APPROXIMATE_SOLUTION) &&
        planning_type_)
    {
        for (size_t attempt = ik_sol_numeral_; attempt < ik_solution_.size(); attempt++)
        {
            LOG_INFO("Need to replan");
            // printPlannerStatus(planner_status);
            // Try next IK solution
            const auto &joint = ik_solution_[attempt];
            for (size_t i = 0; i < planning_group_joints_.size(); i++)
            {
                goal_joint_status_.elements[i] = joint.elements[i];
            }
            if (!checkGoalState(goal_joint_status_, planner_status))
            {
                continue;
            }
            setStartAndGoal();
            res = planner_->solve(solution, planner_status);
            if (res && (planner_status.statuscode == PlannerStatus::PATH_FOUND ||
                        planner_status.statuscode == PlannerStatus::EXACT_SOLUTION ||
                        planner_status.statuscode == PlannerStatus::APPROXIMATE_SOLUTION))
            {
                break;
            }
        }
    }

    auto finish_time = std::chrono::high_resolution_clock::now();
    time_taken = std::chrono::duration<double>(finish_time - start_time).count();

    if (res)
    {
        return true;
    }

    return false;
}

/**
 *
 */
void MotionPlanners::createNamedGroupStates(boost::shared_ptr<srdf::Model> srdf_model)
{
    std::vector<srdf::Model::GroupState> group_states = srdf_model->getGroupStates();
    std::string planning_group_name = config_.planner_config.robot_model_config.planning_group_name;

    for (const auto &group_state : group_states)
    {
        if (group_state.group_ == planning_group_name)
        {
            std::map<std::string, double> named_group_state;

            for (const auto &[joint_name, values] : group_state.joint_values_)
            {
                named_group_state[joint_name] = values[0];
            }

            named_group_states_[group_state.name_] = named_group_state;
        }
    }
}

/**
 *
 */
collision_detection::CollisionLinksName MotionPlanners::getCollidedObjectsNames()
{
    collision_detection::CollisionLinksName collided_links;

    for (const auto &[link1, link2] : collision_object_names_)
    {
        collided_links.collision_link_names.push_back({link1, link2});
    }

    return collided_links;
}

/**
 *
 */
std::vector<std::pair<std::string, std::string>> MotionPlanners::assignDisableCollisionObject(
    const collision_detection::CollisionLinksName &disabled_collision_pair)
{
    std::vector<std::pair<std::string, std::string>> collision_pair;

    for (const auto &pair : disabled_collision_pair.collision_link_names)
    {
        collision_pair.push_back({pair.link_1, pair.link_2});
    }

    return collision_pair;
}
