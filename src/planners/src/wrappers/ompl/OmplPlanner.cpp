#include <wrapper/ompl/OmplPlanner.hpp>

namespace motion_planners
{

OmplPlanner::OmplPlanner()
{
}

OmplPlanner::~OmplPlanner()
{
}

bool OmplPlanner::initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string config_file_path)
{
    LOG_DEBUG_S<<"[OmplPlanner]: OMPL planner initialised";
    robot_model_ = robot_model;      
    
    planning_group_name_ = robot_model_->getPlanningGroupName();         
    LOG_DEBUG_S<<"[OmplPlanner]: OMPL planner initialised for the planning group ="<<planning_group_name_;
    
    planning_group_joints_.clear();
    robot_model_->getPlanningGroupJointinformation(planning_group_name_, planning_group_joints_, chain_link_, tip_link_);
    
    LOG_DEBUG_S<<"[OmplPlanner]: OMPL planner initialised with size = "<<planning_group_joints_.size();
    
    LOG_DEBUG_S<<"[OmplPlanner]: Reading OMPL planner config ";
    // assign the config
    YAML::Node input_config;    
    motion_planners::loadConfigFile(config_file_path, input_config);    
    const YAML::Node& ompl_config_node = input_config["ompl_config"];
    ompl_config_ = handle_ompl_config::getOmplConfig(ompl_config_node);    

    number_of_dimensions_= planning_group_joints_.size();
    hasCartesianConstraint_ = false;

    return true; 
}

void OmplPlanner::setStartGoalTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal)
{    
    start_joint_values_ = start;
    goal_joint_values_ = goal;
}

bool OmplPlanner::updateInitialTrajectory(const base::JointsTrajectory &trajectory)
{}

bool OmplPlanner::checkStartState(const base::samples::Joints &current_robot_status, PlannerStatus &planner_status )
{
    // check whether the start state is in collision
    //this->has_orientation_constraint=false;
    robot_model_->updateJointGroup(current_robot_status);

    if(!robot_model_->isStateValid())
    {
        planner_status.statuscode = PlannerStatus::START_STATE_IN_COLLISION;         
        return false;
    }
    else
    {
        // assign the start joint values from current robot status
        start_joint_values_.clear();
        start_joint_values_.resize(number_of_dimensions_);

        for(size_t i = 0; i < number_of_dimensions_; i++)
        {
            try
            {
                base::JointState current_jointstate = current_robot_status.getElementByName(planning_group_joints_.at(i).first);		

                start_joint_values_.names.at(i) = planning_group_joints_.at(i).first;
                start_joint_values_.elements.at(i) = current_jointstate;

            }
            catch(std::runtime_error const &e) 
            {
                LOG_INFO_S<<"[OmplPlanner]: Cannot get the joint with name %s", planning_group_joints_.at(i).first.c_str();

                return false;
            }
        }
    }
    
    return true;
}

bool OmplPlanner::setUpPlanningTaskInJointSpace(PlannerStatus &planner_status)
{

    std::string joint_name;
    double lower_limit, upper_limit;
    for(std::vector< std::pair<std::string, urdf::Joint> >::iterator it = planning_group_joints_.begin(); it != planning_group_joints_.end(); it++ )
    {
        joint_name                      = it->first;
        lower_limit                     = it->second.limits->lower;
        upper_limit                     = it->second.limits->upper;
        lower_limits_[joint_name]       = lower_limit;
        upper_limits_[joint_name]       = upper_limit;
    }

    if(start_joint_values_.size() != number_of_dimensions_ )
    {
        planner_status.statuscode = PlannerStatus::START_JOINTANGLES_NOT_AVAILABLE;
        return false;
    }

    if( goal_joint_values_.size() != number_of_dimensions_)
    {
        planner_status.statuscode = PlannerStatus::GOAL_JOINTANGLES_NOT_AVAILABLE;
        return false;
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


        LOG_DEBUG_S<<" joint_name ["<<i+1<<"] = " <<joint_name;
        LOG_DEBUG_S<< "joint_space_bounds.low.at(i) " << joint_space_bounds.low.at(i);
        LOG_DEBUG_S<< "joint_space_bounds.high.at(i) " <<joint_space_bounds.high.at(i);
        LOG_DEBUG_S<<"joint_space_start->values[i] "  <<joint_space_start->values[i];
        LOG_DEBUG_S<< "joint_space_goal->values[i] " <<joint_space_goal->values[i];
        LOG_DEBUG_S<<"";
    }
    LOG_DEBUG_S<<"------------------------------------------------------------------------------------" <<std::endl;

    joint_space->as<ompl::base::RealVectorStateSpace>()->setBounds(joint_space_bounds);
    ompl::base::SpaceInformationPtr  joint_space_Space_Information_ptr (new ompl::base::SpaceInformation(joint_space));

    joint_space_Space_Information_ptr->setStateValidityChecker(  boost::bind( &OmplPlanner::collisionCheckerStateValid,this, _1) );

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
        LOG_DEBUG_S<<"[OmplPlanner]: Found solution";

        solved = true;

        ompl::geometric::PathSimplifier path_simplifier(space_information);

        //simplify the solution
        simplifySolution(problem_definition_ptr, path_simplifier, ompl_config_.max_step_smoothing, ompl_config_.max_time_soln_simpilification);

        solution_path_ptr_ = std::static_pointer_cast<ompl::geometric::PathGeometric>(problem_definition_ptr->getSolutionPath());

        solution.resize(number_of_dimensions_, solution_path_ptr_->getStates().size());

        for(size_t i = 0; i < number_of_dimensions_; ++i)
            solution.names.at(i) = planning_group_joints_.at(i).first;

        for(std::size_t i = 0; i < solution_path_ptr_->getStates().size(); i++)
        {
            ompl::base::RealVectorStateSpace::StateType* x=(ompl::base::RealVectorStateSpace::StateType*) solution_path_ptr_->getState(i);

            for(size_t j = 0; j < number_of_dimensions_; j++)
                solution.elements.at(j).at(i).position = x->values[j];    

        }
        std::cout<<"Solution found with size = "<<solution_path_ptr_->getStates().size()<<std::endl;
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

bool OmplPlanner::setUpPlanningTaskInCartesianSpace(PlannerStatus &planner_status)
{

    number_of_dimensions_ = 6;

    double x_start,y_start,z_start,roll_start,pitch_start,yaw_start,x_goal,y_goal,z_goal,roll_goal,pitch_goal,yaw_goal;

    if( (cartesian_contraints_.x.max < cartesian_contraints_.x.min)   || (cartesian_contraints_.y.max < cartesian_contraints_.y.min)   ||
        (cartesian_contraints_.z.max < cartesian_contraints_.z.min)   || (cartesian_contraints_.rx.max < cartesian_contraints_.rx.min) ||
        (cartesian_contraints_.ry.max < cartesian_contraints_.ry.min) || (cartesian_contraints_.rz.max < cartesian_contraints_.rz.min) )
    {
        planner_status.statuscode = PlannerStatus::CONSTRAINED_POSE_NOT_WITHIN_BOUNDS;
        return false;
    }

    lower_limits_["x"]=cartesian_contraints_.x.min;
    upper_limits_["x"]=cartesian_contraints_.x.max;

    lower_limits_["y"]=cartesian_contraints_.y.min;
    upper_limits_["y"]=cartesian_contraints_.y.max;

    lower_limits_["z"]=cartesian_contraints_.z.min;
    upper_limits_["z"]=cartesian_contraints_.z.max;

    lower_limits_["roll"]=cartesian_contraints_.rx.min;
    upper_limits_["roll"]=cartesian_contraints_.rx.max;

    lower_limits_["pitch"]=cartesian_contraints_.ry.min;
    upper_limits_["pitch"]=cartesian_contraints_.ry.max;
    
    lower_limits_["yaw"]=cartesian_contraints_.rz.min;
    upper_limits_["yaw"]=cartesian_contraints_.rz.max;

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
    return robot_model_->isStateValid();

}

bool OmplPlanner::cartesianSpaceStateValidityChecker(const ompl::base::State *state)
{
//     ompl::base::RealVectorStateSpace::StateType* cartesian_pose_to_be_checked=(ompl::base::RealVectorStateSpace::StateType*)state ;
//     KDL::Frame kdl_frame_to_be_checked_in_base_link;
//     double x,y,z,roll, pitch, yaw;
//     x=cartesian_pose_to_be_checked->values[0];
//     y=cartesian_pose_to_be_checked->values[1];
//     z=cartesian_pose_to_be_checked->values[2];
//     roll=cartesian_pose_to_be_checked->values[3];
//     pitch=cartesian_pose_to_be_checked->values[4];
//     yaw=cartesian_pose_to_be_checked->values[5];
//     kdl_frame_to_be_checked_in_base_link.p.x(x);
//     kdl_frame_to_be_checked_in_base_link.p.y(y);
//     kdl_frame_to_be_checked_in_base_link.p.z(z);
//     kdl_frame_to_be_checked_in_base_link.M=kdl_frame_to_be_checked_in_base_link.M.RPY(roll, pitch, yaw);
// 
//     KDL::Frame kdl_frame_to_be_checked_in_kinematic_chain;
//     robot_model_->ConvertPoseBetweenFrames(this->B_Frame,kdl_frame_to_be_checked_in_base_link, this->A_Frame , kdl_frame_to_be_checked_in_kinematic_chain);
// 
// 
//     double roll_, pitch_, yaw_;
// 
//     kdl_frame_to_be_checked_in_kinematic_chain.M.GetRPY(roll_, pitch_, yaw_);    
// 
//     std::map<std::string,double> joints_for_given_pose;
//     std::string planningGroupName=this->motion_planning_request.getplanningGroupName();
//     std::vector<RobotFreeJointParameter> robot_free_joint_parameters=this->motion_planning_request.getRobotFreeJointParameter() ;
// 
//     PlannerStatus ik_result = this->motion_planning_request.getRobotModelptr()->ikSolverUsingIKFAST(    kdl_frame_to_be_checked_in_kinematic_chain,
//                                                                                                         planningGroupName,robot_free_joint_parameters, joints_for_given_pose);
// 
// 
//     if(ik_result.statuscode != PlannerStatus::IK_SUCCESS)
//         return false;
//     else
//         return true;
    
    return true;
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
                path_simplifier.smoothBSpline(static_cast<ompl::geometric::PathGeometric &>(*p), step_size);
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
    if(hasCartesianConstraint_)
    {
        return solveTaskInCartesianSpace(solution, planner_status);
    }
    else
    {
       return solveTaskInJointSpace(solution, planner_status);
    }
}

void OmplPlanner::setCartesianConstraints(CartesianContraints constraints)
{
    cartesian_contraints_ = constraints;
    hasCartesianConstraint_ = true;     
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