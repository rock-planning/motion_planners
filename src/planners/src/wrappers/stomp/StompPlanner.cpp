#include <wrapper/stomp/StompPlanner.hpp>
#include <algorithm>


namespace motion_planners
{
StompPlanner::StompPlanner()
{
}

StompPlanner::~StompPlanner()
{
}

bool StompPlanner::initializePlanner(std::shared_ptr<robot_model::RobotModel>& robot_model, std::string config_file_path)
{

    // assign the config
    YAML::Node input_config;
    motion_planners::loadConfigFile(config_file_path, input_config);
    const YAML::Node& stomp_node = input_config["stomp"];
    const YAML::Node& debug_node = input_config["debug"];
    
    stomp_config_ = handle_stomp_config::getStompConfig(stomp_node);
    debug_config_ = handle_stomp_config::getDebugConfig(debug_node);

    //assigning planning grouup joint names.
    if(!assignPlanningJointInformation(robot_model))
        return false;    

    assert(planning_group_joints_name_.size()==stomp_config_.num_dimensions_);
    optimization_task_.reset(new OptimizationTask(stomp_config_, robot_model_));
    optimization_task_->stompInitialize(1,1);
    
    num_iterations_ = 0;
    
    return true;    
}

bool StompPlanner::reInitializePlanner(const int &num_time_steps)
{
    if(!optimization_task_ || !robot_model_)
    {
        LOG_DEBUG_S<<"[reInitializePlanner] The stomp planner and robot model were not initialised before. This function should be called only if the planner was initialised before";
        return false;
    }

    assert(planning_group_joints_name_.size()==stomp_config_.num_dimensions_);
    stomp_config_.num_time_steps_ = num_time_steps;
    optimization_task_.reset(new OptimizationTask(stomp_config_, robot_model_));
    optimization_task_->stompInitialize(1,1);
    
    num_iterations_ = 0;
    
    return true;

}

bool StompPlanner::solve(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    //optimization_task_->createPolicy();

    optimization_task_->setOptimizationConstraints(constraints_);

    stomp_.reset(new stomp::Stomp());

    stomp_->initialize(stomp_config_, optimization_task_);

    if ((debug_config_.save_noiseless_trajectories_) || (debug_config_.save_noisy_trajectories_))
        mkdir(debug_config_.output_dir_.c_str(), 0755);

    std::stringstream num_rollouts_filename;
    FILE *num_rollouts_file = NULL;
    
    if (debug_config_.save_noisy_trajectories_)
    {
//         std::stringstream stddev_filename;        
        num_rollouts_filename << debug_config_.output_dir_ << "/num_rollouts.txt";
        num_rollouts_file = fopen(num_rollouts_filename.str().c_str(), "w");
    }
    
    if (debug_config_.save_noiseless_trajectories_)
    {
        std::stringstream sss;
        sss << debug_config_.output_dir_ << "/noiseless_0.txt";
        optimization_task_->policy_->writeToFile(sss.str());
	tmp_policy = *optimization_task_->policy_;
    }

    double old_cost = 0.0;
    double cost_improvement = 0.0;
    double current_trajectory_totalcost = 0.0;
    num_iterations_ = 0;
    
    for(int i = 0; i < stomp_config_.num_iterations_; i++)
    {
        num_iterations_++;
        
        stomp_->runSingleIteration(i);

        current_trajectory_totalcost = stomp_->getNoiselessRolloutTotalCost();
        cost_improvement = current_trajectory_totalcost - old_cost;
        old_cost = current_trajectory_totalcost;

        LOG_DEBUG_S <<"Iteration = "<<i <<". Total Cost = "<<current_trajectory_totalcost<<" . Cost improvement = "<<cost_improvement;
        //std::cout <<"Iteration = "<<i <<". Total Cost = "<<current_trajectory_totalcost<<" . Cost improvement = "<<cost_improvement<<std::endl;

        // Stop Criteria 
        // Here the "current_trajectory_totalcost" < 1 means there is no collision.
        // We assign a collision cost of value "1" 
        if((current_trajectory_totalcost < 1 ) && (fabs(cost_improvement) < stomp_config_.min_cost_improvement_))
            break;

        if (debug_config_.save_noisy_trajectories_)
        {
            std::vector<stomp::Rollout> rollouts;
            stomp_->getAllRollouts(rollouts);
            fprintf(num_rollouts_file, "%d\n", int(rollouts.size()));
            for (unsigned int j=0; j<rollouts.size(); ++j)
            {
            std::stringstream ss2;
            ss2 << debug_config_.output_dir_ << "/noisy_" << i+1 << "_" << j << ".txt";
            //tmp_policy.setParameters(rollouts[j].parameters_noise_projected_);
            tmp_policy.setParameters(rollouts[j].parameters_noise_);
            tmp_policy.writeToFile(ss2.str());
            }
        }
        
        if (debug_config_.save_noiseless_trajectories_)
        {
          std::stringstream ss;
          ss << debug_config_.output_dir_ << "/noiseless_" << i+1 << ".txt";
          optimization_task_->policy_->writeToFile(ss.str());          
        }
    }
    
    if (debug_config_.save_noisy_trajectories_)
	fclose(num_rollouts_file);
    
    stomp_.reset();    

    int start = stomp::DIFF_RULE_LENGTH -1;
    int end = (start + stomp_config_.num_time_steps_);
    int diff = (end -start) ;
    //int end = (start + stomp_config_.num_time_steps_ -1);
    //int diff = (end -start) +1;
     
    
    solution.names.resize(planning_group_joints_name_.size());
    solution.elements.resize(planning_group_joints_name_.size());

    for(int d = 0; d < stomp_config_.num_dimensions_; d++)           
    {
	solution.names.at(d) = planning_group_joints_name_.at(d);
	
	//solution.elements.at(d).resize(diff+2);
	solution.elements.at(d).resize(stomp_config_.num_time_steps_+2);
	
	//for( int i = 0; i <= diff+1; i++)	
	for( int i = 0; i < stomp_config_.num_time_steps_+2; i++)	
	    solution.elements.at(d).at(i).position =  optimization_task_->policy_->parameters_all_[d](i+(start-1));
	
    }     

    if ((current_trajectory_totalcost < 1) && (fabs(cost_improvement) <= stomp_config_.min_cost_improvement_))
    {
	planner_status.statuscode = motion_planners::PlannerStatus::PATH_FOUND;
	return true;
    }
    else
	planner_status.statuscode = motion_planners::PlannerStatus::NO_PATH_FOUND;
    
    return false;  
}


void StompPlanner::setStartGoalTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal)
{
    optimization_task_->updateTrajectory(start, goal);
    optimization_task_->input_initial_trajectory_ = optimization_task_->initial_trajectory_;
    
    optimization_task_->createPolicy();   

}

bool StompPlanner::updateInitialTrajectory(const base::JointsTrajectory& trajectory)
{
    if(trajectory.empty())
        return false;

    for (int d=0; d < stomp_config_.num_dimensions_; ++d)
    {
        optimization_task_->initial_trajectory_[d].head(stomp::TRAJECTORY_PADDING) = trajectory.elements.at(d).front().position * 
                                                                                      base::VectorXd::Ones(stomp::TRAJECTORY_PADDING);	
            optimization_task_->initial_trajectory_[d].tail(stomp::TRAJECTORY_PADDING) = trajectory.elements.at(d).back().position  * 
                                                                                         base::VectorXd::Ones(stomp::TRAJECTORY_PADDING);

        for (int i=0; i < stomp_config_.num_time_steps_; i++){
            optimization_task_->initial_trajectory_[d](stomp::TRAJECTORY_PADDING+i) = trajectory.elements.at(d).at(i+1).position;
        }
    }

    optimization_task_->input_initial_trajectory_ = optimization_task_->initial_trajectory_;
    optimization_task_->updatePolicy();   

    return true;
}

base::JointsTrajectory StompPlanner::getInitialTrajectory()
{
    int start = stomp::DIFF_RULE_LENGTH -1;
    int end = (start + stomp_config_.num_time_steps_);
    
    base::JointsTrajectory trajectory;
     
    
    trajectory.names.resize(planning_group_joints_name_.size());
    trajectory.elements.resize(planning_group_joints_name_.size());

    for(int d = 0; d < stomp_config_.num_dimensions_; d++)           
    {
        trajectory.names.at(d) = planning_group_joints_name_.at(d);
        
        //trajectory.elements.at(d).resize(diff+2);
        trajectory.elements.at(d).resize(stomp_config_.num_time_steps_+2);
        
        for( int i = 0; i < stomp_config_.num_time_steps_+2; i++)   
            trajectory.elements.at(d).at(i).position =   optimization_task_->input_initial_trajectory_[d](i+(start-1));
    
    } 
    
    return trajectory;
}

double StompPlanner::getMovementDeltaTime()
{
    if(optimization_task_)
	return optimization_task_->policy_->getMovementDt();
    return 0.0;
}

}// end namespace 
