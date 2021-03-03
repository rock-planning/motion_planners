#include <wrapper/ompl/OmplPlanner.hpp>

namespace motion_planners
{

OmplPlanner::OmplPlanner()
{
    klc_offset_pose_.setIdentity();
    passive_active_offset_.setIdentity();
    planning_group_joints_size_ = 0;
}

OmplPlanner::~OmplPlanner()
{}

bool OmplPlanner::initializePlanner(std::shared_ptr<robot_model::RobotModel>& robot_model, std::string config_file_path)
{
    //assigning planning group joint names.
    if(!assignPlanningJointInformation(robot_model))
        return false;
    
    LOG_DEBUG_S<<"[OmplPlanner]: OMPL planner initialised for the planning group ="<<planning_group_name_;
    LOG_DEBUG_S<<"[OmplPlanner]: OMPL planner initialised with size = "<<planning_group_joints_.size();
    LOG_DEBUG_S<<"[OmplPlanner]: Reading OMPL planner config ";
    // assign the config
    YAML::Node input_config;    
    motion_planners::loadConfigFile(config_file_path, input_config);    
    const YAML::Node& ompl_config_node = input_config["ompl_config"];
    ompl_config_ = handle_ompl_config::getOmplConfig(ompl_config_node);    

    number_of_dimensions_= planning_group_joints_.size();
    planning_group_joints_size_ = planning_group_joints_.size();
    // Joint and Cartesian constraint names
    orientation_constraint_names_   = {"roll", "pitch", "yaw" };
    position_constraint_names_      = {"x", "y", "z" };

    // if only one kinematic solver is available in the robot model, set it as an active chain kinematic solver
    if(robot_model_->robot_kinematics_map_.size() == 1)
    {
        if(!robot_model_->getKinematicsSolver(robot_model_->robot_kinematics_map_.begin()->first, active_chain_kin_solver_))
        return false;
    }

    LOG_DEBUG_S<<"[OmplPlanner]: OMPL planner initialised";
    return true; 
}

void OmplPlanner::setStartGoalTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal)
{    
    start_joint_values_ = start;
    goal_joint_values_ = goal;

    LOG_INFO("[OmplPlanner]: Start joint angles ");    
    for(int i =0; i < start_joint_values_.size(); i++)
        LOG_INFO("[OmplPlanner]: Joint name: %s = %f ",start_joint_values_.names[i].c_str(), start_joint_values_.elements[i].position); 

    LOG_INFO("[OmplPlanner]: Goal joint angles ");
    for(int i =0; i < goal_joint_values_.size(); i++)
        LOG_INFO("[OmplPlanner]: Joint name: %s = %f ",goal_joint_values_.names[i].c_str(), goal_joint_values_.elements[i].position); 
    
}

bool OmplPlanner::updateInitialTrajectory(const base::JointsTrajectory &trajectory)
{}

bool OmplPlanner::setUpPlanningTaskInJointSpace(PlannerStatus &planner_status)
{
   
    // now we assign constraint if available
    if(constraints_.use_constraint == motion_planners::JOINTS_CONSTRAINT)
    {
        if( (constraints_.joint_constraint.value.size() != number_of_dimensions_) ||  
            (constraints_.joint_constraint.tolerance.size() != number_of_dimensions_))
        {
            planner_status.statuscode = PlannerStatus::JOINT_CONSTRAINT_SIZE_ERROR;
            return false;
        }
        for(size_t i = 0; i < number_of_dimensions_; i++)
        {
            lower_limits_[planning_group_joints_.at(i).first] = constraints_.joint_constraint.value(i) - constraints_.joint_constraint.tolerance(i);
            upper_limits_[planning_group_joints_.at(i).first] = constraints_.joint_constraint.value(i) + constraints_.joint_constraint.tolerance(i);
        }
    }
    else
    {
        for(std::vector< std::pair<std::string, urdf::Joint> >::iterator it = planning_group_joints_.begin(); it != planning_group_joints_.end(); it++ )
        {
            lower_limits_[it->first]       = it->second.limits->lower;
            upper_limits_[it->first]       = it->second.limits->upper;
        }
    }    
    return true;    
}

bool OmplPlanner::solveTaskInJointSpace(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    if(!setUpPlanningTaskInJointSpace(planner_status))
    {
        return false;
    }
    
    ompl::base::StateSpacePtr joint_space(new ompl::base::RealVectorStateSpace(number_of_dimensions_));
    ompl::base::RealVectorBounds joint_space_bounds(number_of_dimensions_);

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> joint_space_start(joint_space);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> joint_space_goal(joint_space);

    std::string joint_name;
    LOG_DEBUG_S<<"[OmplPlanner]: SolveTaskInJointSpace";    
    for(std::size_t i = 0; i< number_of_dimensions_; i++ )
    {
        joint_name = planning_group_joints_.at(i).first;
        joint_space_bounds.setLow(i, lower_limits_[joint_name] );
        joint_space_bounds.setHigh(i, upper_limits_[joint_name] );
        joint_space_start->values[i]= start_joint_values_[joint_name].position;
        joint_space_goal->values[i]= goal_joint_values_[joint_name].position;

        LOG_DEBUG_S <<"Joint_name ["<<i+1<<"] = " <<joint_name<<": Lower bound = " << joint_space_bounds.low.at(i)
                    <<"Higher bound = " <<joint_space_bounds.high.at(i)
                    <<"Start value = "  <<joint_space_start->values[i]
                    <<"Goal value = " <<joint_space_goal->values[i];        
    }   

    joint_space->as<ompl::base::RealVectorStateSpace>()->setBounds(joint_space_bounds);
    ompl::base::SpaceInformationPtr  joint_space_Space_Information_ptr (new ompl::base::SpaceInformation(joint_space));

    if(constraints_.use_constraint == motion_planners::KLC_CONSTRAINT)
    {
        // set up the state validity checker
        joint_space_Space_Information_ptr->setStateValidityChecker(  boost::bind( &OmplPlanner::kinematicLoopClosureValidChecker,this, _1) );
    }
    else
        joint_space_Space_Information_ptr->setStateValidityChecker(  boost::bind( &OmplPlanner::jointSpaceStateValidChecker,this, _1) );        

    joint_space_Space_Information_ptr->enforceBounds( joint_space_start.get());
    joint_space_Space_Information_ptr->enforceBounds( joint_space_goal.get());

    ompl::base::ProblemDefinitionPtr problem_definition_ptr(new ompl::base::ProblemDefinition(joint_space_Space_Information_ptr));

    problem_definition_ptr->setStartAndGoalStates(joint_space_start, joint_space_goal);
    ompl::base::PlannerPtr planner_;

    // setting up the planner
    setUpPlanner (planner_, joint_space_Space_Information_ptr, problem_definition_ptr);

    return solveProblem(planner_, joint_space_Space_Information_ptr, problem_definition_ptr, solution, planner_status);
}

void OmplPlanner::setUpPlanner (ompl::base::PlannerPtr &planner, ompl::base::SpaceInformationPtr &space_information, ompl::base::ProblemDefinitionPtr &problem_definition_ptr)
{
    switch (ompl_config_.type_of_planner)
    {
        case ompl_config::SBL :
                planner.reset(new ompl::geometric::SBL (space_information));
                break;
        case  ompl_config::EST:
                planner.reset(new ompl::geometric::EST (space_information));
                break;
        case  ompl_config::LBKPIECE:
                planner.reset(new ompl::geometric::LBKPIECE1 (space_information));
                break;
        case  ompl_config::BKPIECE:
                planner.reset(new ompl::geometric::BKPIECE1 (space_information));
                break;
        case  ompl_config::KPIECE:
                planner.reset(new ompl::geometric::KPIECE1 (space_information));
                break;
        case ompl_config:: RRT:
                planner.reset(new ompl::geometric::RRT (space_information));
                break;
        case  ompl_config::RRTConnect:
                planner.reset(new ompl::geometric::RRTConnect (space_information));
                break;
        case  ompl_config::RRTstar:
                planner.reset(new ompl::geometric::RRTstar (space_information));
                break;
        case  ompl_config::PRM:
                planner.reset(new ompl::geometric::PRM (space_information));
                break;
        case  ompl_config::PRMstar:
                planner.reset(new ompl::geometric::PRMstar(space_information));
                break;
    }
    planner->setProblemDefinition(problem_definition_ptr);
    planner->setup();
}

bool OmplPlanner::solveProblem(ompl::base::PlannerPtr &planner,  const ompl::base::SpaceInformationPtr &space_information, 
                               const ompl::base::ProblemDefinitionPtr &problem_definition_ptr, base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    bool solved = false;
    ompl::base::ParamSet param_set= planner->params();
    param_set.setParam("range",ompl_config_.planner_specific_parameters_range);
    ompl::base::PlannerStatus ompl_planner_status = planner->solve( ompl_config_.max_solve_time);

    if(ompl_planner_status)    
    {
        solved = true;
        solution_path_ptr_ = std::static_pointer_cast<ompl::geometric::PathGeometric>(problem_definition_ptr->getSolutionPath());
        
        LOG_DEBUG_S<<"[OmplPlanner]: Found solution with size "<<solution_path_ptr_->getStates().size();
        std::cout<<"Solution found before simplifying with size = "<<solution_path_ptr_->getStates().size()<<std::endl;
        
        ompl::geometric::PathSimplifier path_simplifier(space_information);
        if(constraints_.use_constraint != motion_planners::KLC_CONSTRAINT )
        {            
            //simplify the solution
            simplifySolution(problem_definition_ptr, path_simplifier, ompl_config_.max_step_smoothing, ompl_config_.max_time_soln_simpilification);
            
            // Single arm planner
            solution.resize(planning_group_joints_size_, solution_path_ptr_->getStates().size());
            for(size_t i = 0; i < planning_group_joints_size_; ++i)
                solution.names.at(i) = planning_group_joints_.at(i).first;
        }
        else
        {        
            // dual arm planner
            int dual_arm_sol_size = planning_group_joints_size_+ passive_chain_solution_.names.size();
            solution.resize(dual_arm_sol_size, solution_path_ptr_->getStates().size());

            for(size_t i = 0; i < planning_group_joints_size_; ++i)
                solution.names.at(i) = planning_group_joints_.at(i).first;

            for(size_t i = planning_group_joints_size_; i < dual_arm_sol_size; ++i)
                solution.names.at(i) = passive_chain_solution_.names.at(i-planning_group_joints_size_);
        }
                
        for(std::size_t i = 0; i < solution_path_ptr_->getStates().size(); i++)
        {
            ompl::base::RealVectorStateSpace::StateType* x=(ompl::base::RealVectorStateSpace::StateType*) solution_path_ptr_->getState(i);           
            
            if(constraints_.use_constraint != motion_planners::POSE_CONSTRAINT)
            {   
                for(size_t j = 0; j < planning_group_joints_size_; j++)
                    solution.elements.at(j).at(i).position = x->values[j];
                // we need to go through the simplified soln and get the passive joint
                // No need to check the collision as it was checked while simplifying the solution
                if(constraints_.use_constraint == motion_planners::KLC_CONSTRAINT)
                {
                    // convert the ompl state to base joint type
                    base::samples::Joints active_joint_values;
                    convertOmplStateToBaseJoints(solution_path_ptr_->getState(i), active_joint_values); 

                    std::vector<base::commands::Joints> passive_chain_iksoln;
                    PlannerStatus planner_status;
                    if(!calculatePassiveChainIKSoln(  active_joint_values, passive_chain_projected_state_.front(), 
                                                  passive_chain_iksoln, planner_status))
                    {
                        std::cout<<"[OmplPlanner::solveProblem] This should not happend :("<<std::endl;
                        return false;
                    }  
                    // store the passive chain solution
                    for(std::size_t ii = 0; ii < passive_chain_iksoln[0].names.size(); ii++)
                    {
                        solution.elements.at(planning_group_joints_size_+ii).at(i).position = passive_chain_iksoln[0].elements[ii].position;                        
                    }
                }
            }
            else
            {                
                base::samples::RigidBodyState target_pose;
                target_pose.sourceFrame = start_pose_.sourceFrame;
                target_pose.targetFrame = start_pose_.targetFrame;
                
                target_pose.position.x() = x->values[0];
                target_pose.position.y() = x->values[1];
                target_pose.position.z() = x->values[2];
                target_pose.orientation = base::Quaterniond(  base::AngleAxisd(x->values[5], Eigen::Matrix<double,3,1>::UnitZ())*
                                                              base::AngleAxisd(x->values[4], Eigen::Matrix<double,3,1>::UnitY())*
                                                              base::AngleAxisd(x->values[3], Eigen::Matrix<double,3,1>::UnitX()));
                
                std::vector<base::commands::Joints> ik_solution;
                kinematics_library::KinematicsStatus solver_status;

                active_chain_kin_solver_->solveIK(target_pose, start_joint_values_, ik_solution, solver_status);
                
                if(solver_status.statuscode != kinematics_library::KinematicsStatus::IK_FOUND)
                    return false;
                
                for(size_t j = 0; j < planning_group_joints_size_; j++)
                    solution.elements.at(j).at(i).position = ik_solution[0].elements.at(j).position;                   
            }   
        }        
        planner_status.statuscode = motion_planners::PlannerStatus::PATH_FOUND;
    }
    else
    {
        planner_status.statuscode = motion_planners::PlannerStatus::NO_PATH_FOUND;
    }
    // set the planner status
    setPlannerStatus(ompl_planner_status, planner_status);

    return solved;
}


bool OmplPlanner::setLimitsFromConstraints( const std::vector<std::string> &value_name, 
                                            const ConstraintValues &constraint)
{
    if( (value_name.size() != constraint.value.size()) || ( constraint.value.size() != constraint.tolerance.size()) || 
        ( value_name.size() != constraint.tolerance.size()) )
        return false;

    for(std::size_t i = 0; i < value_name.size(); i++)
    {
        lower_limits_[value_name[i]]  = constraint.value(i) - constraint.tolerance(i);
        upper_limits_[value_name[i]]  = constraint.value(i) + constraint.tolerance(i);
    }
    return true;
}

bool OmplPlanner::setUpPlanningTaskInCartesianSpace(PlannerStatus &planner_status)
{
    // 6 dof - 3 Position + 3 orientation
    number_of_dimensions_ = 6;

    double x_start,y_start,z_start,roll_start,pitch_start,yaw_start,x_goal,y_goal,z_goal,roll_goal,pitch_goal,yaw_goal;

    lower_limits_["x"] = -5.0; upper_limits_["x"] = 5.0; lower_limits_["y"] = -5.0; upper_limits_["y"] = 5.0;
    lower_limits_["z"] = -5.0; upper_limits_["z"] = 5.0;
    lower_limits_["roll"] = -3.145; upper_limits_["roll"] = 3.145;lower_limits_["pitch"] = -3.145; upper_limits_["pitch"] = 3.145;
    lower_limits_["yaw"] = -3.145; upper_limits_["yaw"] = 3.145;

    if(constraints_.use_constraint == motion_planners::ORIENTATION_CONSTRAINT)
    {
        setLimitsFromConstraints( orientation_constraint_names_, constraints_.orientation_constraint);
    }
    else if(constraints_.use_constraint == motion_planners::POSITION_CONSTRAINT)
    {
        setLimitsFromConstraints( position_constraint_names_, constraints_.position_constraint);
    }
    else if(constraints_.use_constraint == motion_planners::POSE_CONSTRAINT)
    {
        setLimitsFromConstraints( position_constraint_names_, constraints_.position_constraint);
        setLimitsFromConstraints( orientation_constraint_names_, constraints_.orientation_constraint);
    }
    
    // assigning start and goal pose
    kinematics_library::KinematicsStatus solver_status;
    
    if(!active_chain_kin_solver_->solveFK(start_joint_values_, start_pose_, solver_status))    
        return false;
    
    if(!active_chain_kin_solver_->solveFK(goal_joint_values_, goal_pose_, solver_status))    
        return false;

    return true;
}

bool OmplPlanner::jointSpaceStateValidChecker(const ompl::base::State *state)
{

    ompl::base::RealVectorStateSpace::StateType* joint_values_to_be_checked = (ompl::base::RealVectorStateSpace::StateType*)state ;
    base::samples::Joints joint_values;
    joint_values.resize(planning_group_joints_name_.size());
    
    joint_values.names = planning_group_joints_name_;
        
    for(int i = 0; i < planning_group_joints_name_.size(); i++)
    {
        joint_values.elements.at(i).position = joint_values_to_be_checked->values[i];        
    }
    // checking for collision
    robot_model_->updateJointGroup(joint_values);
    
    if(!robot_model_->isStateValid(collision_cost_))
        return false;

    return true;
}

bool OmplPlanner::kinematicLoopClosureProjection(   const base::samples::Joints &joint_values, 
                                                    std::vector<base::commands::Joints> &projected_state)
{    
    // std::cout<<"kinematicLoopClosureProjection"<<std::endl;
    projected_state.clear();
    
    PlannerStatus planner_status;
    if(!calculatePassiveChainIKSoln( joint_values, passive_chain_projected_state_.back(), projected_state, planner_status))
        return false;

    // now update the active chain
    robot_model_->updateJointGroup(joint_values);
    // now update the passiveactive chain
    // TODO: Currently using the optimal ik solution, maybe need to go through all the solutions for CC.
    robot_model_->updateJointGroup(projected_state[0]);
    
    // checking for collision
    if(!robot_model_->isStateValid(collision_cost_))
        return false;

    // push back the ik solution, which will be used by IK as initial seed / pick opt ik soln 
    passive_chain_projected_state_.push_back(projected_state[0]);

    return true;
}

bool OmplPlanner::calculatePassiveChainIKSoln(  const base::samples::Joints &active_chain_joints, const base::samples::Joints &passive_chain_joints, 
                                                std::vector<base::commands::Joints> &passive_chain_iksoln, PlannerStatus &planner_status)
{

    base::samples::RigidBodyState active_chain_pose, passive_chain_pose;
     
    // do forward kinematic(FK) for the active chain    
    if(!active_chain_kin_solver_->solveFK(active_chain_joints, active_chain_pose, planner_status.kinematic_status))        
        return false;
    // calculate the passive chain pose using the offset and active chain
    Eigen::Affine3d  rTa, rTp;    
    rTa = active_chain_pose.getTransform();
    rTp = passive_active_offset_ * rTa * klc_offset_pose_;
    // do IK for the passive pose
    passive_chain_pose.setTransform(rTp);
 
    passive_chain_pose.sourceFrame = passive_chain_pose_.sourceFrame;
    passive_chain_pose.targetFrame = passive_chain_pose_.targetFrame;
    std::vector<base::commands::Joints> ik_solution;

    if(!passive_chain_kin_solver_->solveIK(passive_chain_pose, passive_chain_joints, passive_chain_iksoln, planner_status.kinematic_status))
    {
        return false;
    }

    return true;
}

bool OmplPlanner::checkPassiveChainCollision(PlannerStatus &planner_status)
{
    // get the joint angles for the passive chain
    base::samples::Joints passive_chain_joints;        
    if(!robot_model_->getPlanningGroupJointInformation(constraints_.klc_constraint.passive_chain_planning_group, 
                                                        passive_chain_joints))
    {
        LOG_ERROR_S<<"[OmplPlanner]: Cannot find planning group "<<constraints_.klc_constraint.passive_chain_planning_group.c_str();
    }

    std::vector<base::commands::Joints> passive_chain_iksoln;
    if(!calculatePassiveChainIKSoln( goal_joint_values_, passive_chain_joints, passive_chain_iksoln, planner_status))
    {
        planner_status.statuscode = PlannerStatus::KINEMATIC_ERROR;
        planner_status.kinematic_status.statuscode = kinematics_library::KinematicsStatus::NO_IK_SOLUTION;
        return false;
    }

    robot_model_->updateJointGroup(passive_chain_iksoln[0]);
    if(!robot_model_->isStateValid(collision_cost_))
    {
        planner_status.statuscode = PlannerStatus::GOAL_STATE_IN_COLLISION;
        return false;
    }

    return true;
}


void OmplPlanner::convertOmplStateToBaseJoints(const ompl::base::State *state, base::samples::Joints &joint_values)
{
    ompl::base::RealVectorStateSpace::StateType* joint_values_to_be_checked = (ompl::base::RealVectorStateSpace::StateType*)state ;
    // convert the ompl state to base samples joint
    joint_values.resize(planning_group_joints_name_.size());    
    joint_values.names = planning_group_joints_name_;        
    for(int i = 0; i < planning_group_joints_name_.size(); i++)
    {
        joint_values.elements.at(i).position = joint_values_to_be_checked->values[i];        
    }
}


bool OmplPlanner::kinematicLoopClosureValidChecker(const ompl::base::State *state)
{
    // std::cout<<"kinematicLoopClosureValidChecker"<<std::endl;
    // convert the ompl state to base joint type
    base::samples::Joints joint_values;
    convertOmplStateToBaseJoints(state, joint_values) ;  

    // project the passive chain value in active chain manifold.
    std::vector<base::commands::Joints> ik_solution;

    if(!kinematicLoopClosureProjection( joint_values, ik_solution))
        return false;    

    return true;
}

bool OmplPlanner::collisionCheckerStateValid(const ompl::base::State *state)
{

    ompl::base::RealVectorStateSpace::StateType* joint_values_to_be_checked=(ompl::base::RealVectorStateSpace::StateType*)state ;
    std::map<std::string, double> joint_state_map;
    std::string joint_name;
    double joint_value;

    int i=0;

    for(std::vector< std::pair <std::string, urdf::Joint> >::iterator it= planning_group_joints_.begin();it!= planning_group_joints_.end()  ;it++)
    {
        joint_name=it->first;
        joint_value=joint_values_to_be_checked->values[i];
        joint_state_map[joint_name]=joint_value;
        i++;
    }

    robot_model_->updateJointGroup(joint_state_map);
    return robot_model_->isStateValid(collision_cost_);

}

bool OmplPlanner::cartesianSpaceStateValidityChecker(const ompl::base::State *state)
{    
    ompl::base::RealVectorStateSpace::StateType* cartesian_pose_to_be_checked = (ompl::base::RealVectorStateSpace::StateType*)state ;
    
    base::samples::RigidBodyState target_pose;
    target_pose.sourceFrame = start_pose_.sourceFrame;
    target_pose.targetFrame = start_pose_.targetFrame;
    
    target_pose.position.x() = cartesian_pose_to_be_checked->values[0];
    target_pose.position.y() = cartesian_pose_to_be_checked->values[1];
    target_pose.position.z() = cartesian_pose_to_be_checked->values[2];
    
    target_pose.orientation = base::Quaterniond(base::AngleAxisd(cartesian_pose_to_be_checked->values[5], Eigen::Matrix<double,3,1>::UnitZ())*
                                                base::AngleAxisd(cartesian_pose_to_be_checked->values[4], Eigen::Matrix<double,3,1>::UnitY())*
                                                base::AngleAxisd(cartesian_pose_to_be_checked->values[3], Eigen::Matrix<double,3,1>::UnitX()));    
    
    std::vector<base::commands::Joints> ik_solution;
    kinematics_library::KinematicsStatus solver_status;

    active_chain_kin_solver_->solveIK(target_pose, start_joint_values_, ik_solution, solver_status);

    if(solver_status.statuscode != kinematics_library::KinematicsStatus::IK_FOUND)
        return false;
    
    robot_model_->updateJointGroup(ik_solution[0]);
    return robot_model_->isStateValid(collision_cost_);
}

bool OmplPlanner::solveTaskInCartesianSpace(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    if(!setUpPlanningTaskInCartesianSpace(planner_status))
    {
        return false;
    }

    bool solved;
    ompl::base::StateSpacePtr cartesian_space(new ompl::base::RealVectorStateSpace(number_of_dimensions_));
    ompl::base::RealVectorBounds cartesian_space_bounds(number_of_dimensions_);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> cartesian_space_start(cartesian_space);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> cartesian_space_goal(cartesian_space);

    cartesian_space_bounds.setLow(0, lower_limits_["x"] );
    cartesian_space_bounds.setHigh(0 , upper_limits_["x"] );
    cartesian_space_start->values[0] = start_pose_.position.x();
    cartesian_space_goal->values[0] = goal_pose_.position.x();

    cartesian_space_bounds.setLow(1, lower_limits_["y"] );
    cartesian_space_bounds.setHigh(1, upper_limits_["y"] );
    cartesian_space_start->values[1] = start_pose_.position.y();
    cartesian_space_goal->values[1] = goal_pose_.position.y();

    cartesian_space_bounds.setLow(2, lower_limits_["z"] );
    cartesian_space_bounds.setHigh(2 ,upper_limits_["z"] );
    cartesian_space_start->values[2] = start_pose_.position.z();
    cartesian_space_goal->values[2] = goal_pose_.position.z();

    cartesian_space_bounds.setLow(3, lower_limits_["roll"] );
    cartesian_space_bounds.setHigh(3, upper_limits_["roll"] );
    cartesian_space_start->values[3] = start_pose_.getRoll();
    cartesian_space_goal->values[3] = goal_pose_.getRoll();

    cartesian_space_bounds.setLow(4, lower_limits_["pitch"] );
    cartesian_space_bounds.setHigh(4 , upper_limits_["pitch"] );
    cartesian_space_start->values[4]= start_pose_.getPitch();
    cartesian_space_goal->values[4]= goal_pose_.getPitch();

    cartesian_space_bounds.setLow(5, lower_limits_["yaw"] );
    cartesian_space_bounds.setHigh(5 , upper_limits_["yaw"] );
    cartesian_space_start->values[5] = start_pose_.getYaw();
    cartesian_space_goal->values[5] = goal_pose_.getYaw();
 
    cartesian_space->as<ompl::base::RealVectorStateSpace>()->setBounds(cartesian_space_bounds);
    ompl::base::SpaceInformationPtr  cartesian_space_Space_Information_ptr (new ompl::base::SpaceInformation(cartesian_space));

    cartesian_space_Space_Information_ptr->setStateValidityChecker(  boost::bind( &OmplPlanner::cartesianSpaceStateValidityChecker,this, _1) );

    cartesian_space_Space_Information_ptr->enforceBounds( cartesian_space_start.get());
    cartesian_space_Space_Information_ptr->enforceBounds( cartesian_space_goal.get());

    ompl::base::ProblemDefinitionPtr problem_definition_ptr(new ompl::base::ProblemDefinition(cartesian_space_Space_Information_ptr));

    problem_definition_ptr->setStartAndGoalStates(cartesian_space_start, cartesian_space_goal);
    ompl::base::PlannerPtr planner_;

    // setting up the planner
    setUpPlanner (planner_, cartesian_space_Space_Information_ptr, problem_definition_ptr);

    return solveProblem(planner_, cartesian_space_Space_Information_ptr, problem_definition_ptr, solution, planner_status);

}

// This function is copied from ompl's SimpleSetup 
void OmplPlanner::simplifySolution(const ompl::base::ProblemDefinitionPtr &problem, ompl::geometric::PathSimplifier &path_simplifier,
                                   unsigned int step_size, double duration)
{

    if (problem)
    {
        const ompl::base::PathPtr &p = problem->getSolutionPath();

        if (p)
        {
            ompl::time::point start = ompl::time::now();
            auto &path = static_cast<ompl::geometric::PathGeometric &>(*p);
            std::size_t num_states = path.getStateCount();
            if (duration < std::numeric_limits<double>::epsilon())
                path_simplifier.simplifyMax(static_cast<ompl::geometric::PathGeometric &>(*p));
            else
            {
                path_simplifier.simplify(static_cast<ompl::geometric::PathGeometric &>(*p), duration);
                LOG_INFO("[OmplPlanner]: Finished simplifying");
                path_simplifier.smoothBSpline(static_cast<ompl::geometric::PathGeometric &>(*p), step_size);
                LOG_INFO("[OmplPlanner]: Finished smoothing");
            }

            double simplify_time = ompl::time::seconds(ompl::time::now() - start);
            LOG_INFO("[OmplPlanner]: Path simplification took %f seconds and changed from %d to %d states",
            simplify_time, num_states, path.getStateCount());

            return;
        }
    }
    LOG_WARN_S<<"[OmplPlanner]: No solution to simplify";
}

bool OmplPlanner::solve(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    if( (constraints_.use_constraint == motion_planners::NO_CONSTRAINT) || 
        (constraints_.use_constraint == motion_planners::JOINTS_CONSTRAINT))
    {
        return solveTaskInJointSpace(solution, planner_status);
    }
    else if(constraints_.use_constraint == motion_planners::KLC_CONSTRAINT)
    {
        passive_chain_projected_state_.clear();

        // calculate the KLC offset pose
        if(!calculateKLCOffset())
            return false;
        // check collision for passive chain goal pose
        if(!checkPassiveChainCollision(planner_status))
            return false;
        
        return solveTaskInJointSpace(solution, planner_status);
    }
    else
    {
        return solveTaskInCartesianSpace(solution, planner_status);
    }
}

bool OmplPlanner::calculateKLCOffset()
{
    // calculate the offset
    // rTp = rTa x aTp
    // aTp = aTr x rTp
    // rTp = Transformation of root frame to passive chain end-eff
    // rTa = Transformation of root frame to active chain end-eff
    // aTp = Offset from active chain end-eff to passive chain end-eff. This offset is the constraint

    // get the active and passive chain kinematic solver
    if(!robot_model_->getKinematicsSolver(constraints_.klc_constraint.active_chain_kinematic_name, active_chain_kin_solver_))    
        return false;
    if(!robot_model_->getKinematicsSolver(constraints_.klc_constraint.passive_chain_kinematic_name, passive_chain_kin_solver_))    
        return false;
    //update the joint group
    robot_model_->updateJointGroup(start_joint_values_);

    // do forward kinematic(FK) for the active chain
    kinematics_library::KinematicsStatus solver_status;    
    if(!active_chain_kin_solver_->solveFK(start_joint_values_, active_chain_pose_, solver_status))        
        return false;

    // get the joint angles for the passive chain and do the FK.
    base::samples::Joints passive_chain_joints;        
    if(!robot_model_->getPlanningGroupJointInformation(constraints_.klc_constraint.passive_chain_planning_group, 
                                                        passive_chain_joints))
    {
        LOG_ERROR_S<<"[OmplPlanner]: Cannot find planning group "<<constraints_.klc_constraint.passive_chain_planning_group.c_str();
    }

    //initialise the passive chain trajectory
    passive_chain_solution_.clear();
    passive_chain_solution_.resize(passive_chain_joints.size());
    passive_chain_solution_.names = passive_chain_joints.names;

    // do forward kinematic(FK) for the passive chain      
    if(!passive_chain_kin_solver_->solveFK(passive_chain_joints, passive_chain_pose_, solver_status))
    {
        return false;    
    }

    // get the offset between the passive to chain base frame
    passive_active_offset_ = passive_chain_kin_solver_->transformPose( passive_chain_pose_.sourceFrame, active_chain_pose_.sourceFrame);

    Eigen::Affine3d  rTp, rTa;
    rTa = active_chain_pose_.getTransform();
    rTp = passive_chain_pose_.getTransform();
    // Offset from active chain end-eff to passive chain end-eff. This offset is the constraint    
    klc_offset_pose_ = (passive_active_offset_*rTa).inverse()  * rTp;

    //assign the current state
    passive_chain_projected_state_.push_back(passive_chain_joints);

    // std::cout<<"Active chain start Affine \n"<<rTa.translation()<<std::endl;
    // std::cout<<"Passive chain start Affine \n"<<rTp.translation()<<std::endl;
    //  std::cout<<"passive_active_offset_ offset\n"<<passive_active_offset_.translation()<<std::endl;
    //  std::cout<<"KLC offset\n"<<klc_offset_pose_.translation()<<std::endl;
    //  std::cout<<"KLC offset\n"<<klc_offset_pose_.rotation()<<std::endl;

    return true;
}

void OmplPlanner::setPlannerStatus(const ompl::base::PlannerStatus::StatusType &ompl_status, motion_planners::PlannerStatus &planner_status)
{
    switch(ompl_status)
    {
        case ompl::base::PlannerStatus::PlannerStatus::INVALID_START:
            planner_status.statuscode = motion_planners::PlannerStatus::INVALID_START_STATE; break;
        case ompl::base::PlannerStatus::PlannerStatus::INVALID_GOAL:
            planner_status.statuscode = motion_planners::PlannerStatus::INVALID_GOAL_STATE; break;
        case ompl::base::PlannerStatus::PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
            planner_status.statuscode = motion_planners::PlannerStatus::UNRECOGNIZED_GOAL_TYPE; break;
        case ompl::base::PlannerStatus::PlannerStatus::TIMEOUT:
            planner_status.statuscode = motion_planners::PlannerStatus::TIMEOUT; break;
        case ompl::base::PlannerStatus::PlannerStatus::APPROXIMATE_SOLUTION:
            planner_status.statuscode = motion_planners::PlannerStatus::APPROXIMATE_SOLUTION; break;
        case ompl::base::PlannerStatus::PlannerStatus::EXACT_SOLUTION:
            planner_status.statuscode = motion_planners::PlannerStatus::EXACT_SOLUTION; break;
        case ompl::base::PlannerStatus::PlannerStatus::CRASH:
            planner_status.statuscode = motion_planners::PlannerStatus::CRASH; break;
        default:
        {
            LOG_WARN_S<<"[OmplPlanner]: This state is not added "<<ompl_status;
            //throw new std::runtime_error("This planner status is unknown");
        }
    }
}

}// end namespace motion_planners
