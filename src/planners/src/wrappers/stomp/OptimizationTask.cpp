#include <wrapper/stomp/OptimizationTask.hpp>
#include <algorithm>


namespace motion_planners
{

OptimizationTask::OptimizationTask(stomp::StompConfig config, std::shared_ptr<robot_model::RobotModel>& robot_model):stomp_config_(config), robot_model_(robot_model)
{
    
    planning_group_name_ = robot_model_->getPlanningGroupName();
    robot_model_->getPlanningGroupJointsName(planning_group_name_, planning_group_joints_names_);
    if(!robot_model->getJointLimits(lower_limits_, upper_limits_))
	LOG_FATAL_S<<"[OptimizationTask]: Cannot get joint limits";
}

OptimizationTask::~OptimizationTask()
{
    policy_.reset();
}

bool OptimizationTask::stompInitialize(int num_threads, int num_rollouts)
{    
    derivative_costs_.clear();
    derivative_costs_.resize(stomp_config_.num_dimensions_, 
    base::MatrixXd::Zero(stomp_config_.num_time_steps_ + 2 * stomp::TRAJECTORY_PADDING, stomp::NUM_DIFF_RULES));
    
    initial_trajectory_.clear();
    initial_trajectory_.resize(stomp_config_.num_dimensions_, 
    base::VectorXd::Zero(stomp_config_.num_time_steps_ + 2 * stomp::TRAJECTORY_PADDING));

    for (int d=0; d < stomp_config_.num_dimensions_; ++d)
        derivative_costs_[d].col(stomp::STOMP_ACCELERATION) = base::VectorXd::Ones(stomp_config_.num_time_steps_ + 2 * stomp::TRAJECTORY_PADDING);

    proj_pos_.resize(stomp_config_.num_dimensions_, stomp_config_.num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
    pos_.resize(stomp_config_.num_dimensions_, stomp_config_.num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
    vel_.resize(stomp_config_.num_dimensions_, stomp_config_.num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
    acc_.resize(stomp_config_.num_dimensions_, stomp_config_.num_time_steps_ + 2*stomp::TRAJECTORY_PADDING);
    
    collision_costs_.resize(0);
    collision_costs_ = base::VectorXd::Zero(stomp_config_.num_time_steps_);

    return true;
}

void OptimizationTask::updateTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal)
{
    double increment = 0;   

    for (int d=0; d < stomp_config_.num_dimensions_; ++d)
    {
        initial_trajectory_[d].head(stomp::TRAJECTORY_PADDING) 	= 1.0 * start.elements.at(d).position * base::VectorXd::Ones(stomp::TRAJECTORY_PADDING);
        initial_trajectory_[d].tail(stomp::TRAJECTORY_PADDING) 	= 1.0 * goal.elements.at(d).position  * base::VectorXd::Ones(stomp::TRAJECTORY_PADDING); 

        increment = (goal.elements.at(d).position - start.elements.at(d).position)/(stomp_config_.num_time_steps_ - 1);

        for (int i=0; i < stomp_config_.num_time_steps_; i++)
            initial_trajectory_[d](stomp::TRAJECTORY_PADDING+i) = start.elements.at(d).position+(i*increment);		
    } 
}

bool OptimizationTask::getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy)
{
    policy = policy_;
    return true;
}

bool OptimizationTask::setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy)
{
    policy_ = policy;
    return true;
}

double OptimizationTask::getControlCostWeight()
{
    return stomp_config_.control_cost_weight_;
}

bool OptimizationTask::filter(std::vector<base::VectorXd>& parameters, int rollout_id, int thread_id) 
{  
    bool filtered = false;

    for (unsigned int d=0; d<parameters.size(); ++d)
    {
        for (int t=0; t<stomp_config_.num_time_steps_; ++t)
        {
            if (parameters[d](t) < lower_limits_.at(d))
            {
                parameters[d](t) = lower_limits_.at(d);
                filtered = true;
            }
            if (parameters[d](t) > upper_limits_.at(d))
            {
                parameters[d](t) = upper_limits_.at(d);
                filtered = true;
            }
        }
    }
    return filtered;
}

void OptimizationTask::createPolicy()
{
    policy_.reset(new stomp::CovariantMovementPrimitive());
    policy_->initialize(stomp_config_.num_time_steps_, stomp_config_.num_dimensions_, stomp_config_.movement_duration_, derivative_costs_, initial_trajectory_);
    policy_->setToMinControlCost();
    policy_->getParametersAll(initial_trajectory_);
    vel_diff_matrix_ = policy_->getDifferentiationMatrix(stomp::STOMP_VELOCITY);
    acc_diff_matrix_ = policy_->getDifferentiationMatrix(stomp::STOMP_ACCELERATION);

    movement_dt_ = policy_->getMovementDt();

}

void OptimizationTask::updatePolicy()
{
    policy_.reset(new stomp::CovariantMovementPrimitive());
    policy_->initialize(stomp_config_.num_time_steps_, stomp_config_.num_dimensions_, stomp_config_.movement_duration_, derivative_costs_, initial_trajectory_);
    policy_->setParametersAll(initial_trajectory_);
    policy_->updateMinControlCostParameters(initial_trajectory_);
    vel_diff_matrix_ = policy_->getDifferentiationMatrix(stomp::STOMP_VELOCITY);
    acc_diff_matrix_ = policy_->getDifferentiationMatrix(stomp::STOMP_ACCELERATION);

    movement_dt_ = policy_->getMovementDt();

}

bool OptimizationTask::execute( std::vector<base::VectorXd>& parameters,
                                std::vector<base::VectorXd>& projected_parameters,
                                base::VectorXd& costs,
                                base::MatrixXd& weighted_feature_values,
                                const int iteration_number,
                                const int rollout_number,
                                int thread_id,
                                bool compute_gradients,
                                std::vector<base::VectorXd>& gradients,
                                bool& validity)

{

    costs = base::VectorXd::Zero(stomp_config_.num_time_steps_);
    collision_costs_.setZero();
    validity = true;


    for (int d=0; d<stomp_config_.num_dimensions_; ++d)
    {
        proj_pos_.row(d) = initial_trajectory_[d];
        pos_.row(d) = initial_trajectory_[d];
        //printf("lvalue size = %d, %d, rvalue size = %d, %d", 1, stomp_config.num_time_steps_, projected_parameters[d].rows(), projected_parameters[d].cols());
        proj_pos_.block(d, stomp::TRAJECTORY_PADDING, 1, stomp_config_.num_time_steps_) = projected_parameters[d].transpose();
        pos_.block(d, stomp::TRAJECTORY_PADDING, 1, stomp_config_.num_time_steps_) = parameters[d].transpose();

    }

    // calculate collision cost
    computeCollisionCost(costs, validity);
    // calculate orientation cost
    if(constraints_.use_orientation_constraint)
        computeOrientationConstraintCost(costs);

    //auto finish_time = std::chrono::high_resolution_clock::now();
    //std::chrono::duration<double> elapsed = finish_time - start_time;
    //std::cout << "Elapsed time colli: " << elapsed.count() << " s\n";    
    //costs = collision_costs_;    
    //std::cout<<"COST = "<<costs<<std::endl;

    return true;    
}

void OptimizationTask::computeCollisionCost( base::VectorXd& costs, bool& validity)
{
    double collision_cost=0.0;
    
    for (int t = stomp::TRAJECTORY_PADDING; t < stomp::TRAJECTORY_PADDING + stomp_config_.num_time_steps_; ++t)
    {
        // State cost
        robot_model_->updateJointGroup(planning_group_joints_names_, pos_.block(0,t,stomp_config_.num_dimensions_,1));

        if(!robot_model_->isStateValid())
        {
            collision_cost = 1.0;
            validity = false;
        }
        else
        {
            collision_cost = 0.0;
            validity = true;
        }
        costs(t-stomp::TRAJECTORY_PADDING) = collision_cost;

    }

}

void OptimizationTask::computeOrientationConstraintCost( base::VectorXd& costs)
{
    double orientation_cost=0.0;
    kinematics_library::KinematicsStatus kinematic_status;
    
    for (int t = stomp::TRAJECTORY_PADDING; t < stomp::TRAJECTORY_PADDING + stomp_config_.num_time_steps_; ++t)
    {
        std::vector<double> jt_angle(planning_group_joints_names_.size());
        VectorXd::Map(&jt_angle[0], pos_.block(0,t,stomp_config_.num_dimensions_,1).size()) = pos_.block(0,t,stomp_config_.num_dimensions_,1);
        base::samples::Joints current_jt = base::samples::Joints::Positions(jt_angle, planning_group_joints_names_);
        base::samples::RigidBodyState fk_pose;                
        robot_model_->robot_kinematics_->solveFK(current_jt, fk_pose, kinematic_status );
        base::Vector3d euler_angle;
        kinematics_library::quaternionToEuler(fk_pose.orientation, euler_angle);
        double delta_roll       = constraints_.orientation_constraint_tolerance.x() - fabs(constraints_.orientation_constraint.x() - euler_angle.x());
        double delta_pitch      = constraints_.orientation_constraint_tolerance.y() - fabs(constraints_.orientation_constraint.y() - euler_angle.y());
        double delta_yaw        = constraints_.orientation_constraint_tolerance.z() - fabs(constraints_.orientation_constraint.z() - euler_angle.z());
        
        if(delta_roll < 0.0)
            orientation_cost = orientation_cost + (-1.0 * delta_roll);
        if(delta_pitch < 0.0)
            orientation_cost = orientation_cost + (-1.0 * delta_pitch);
        if(delta_yaw < 0.0)
            orientation_cost = orientation_cost + (-1.0 * delta_yaw);
        double orientation_contraint_weight = 2.0;
        orientation_cost = orientation_contraint_weight * (orientation_cost + 1.0);
        costs(t-stomp::TRAJECTORY_PADDING) = costs(t-stomp::TRAJECTORY_PADDING) + orientation_cost;

    }

}





}
// end namespace 
