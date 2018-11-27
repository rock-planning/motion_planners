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

bool StompPlanner::initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string config_file_path)
{    
    robot_model_ = robot_model;      
    
    planning_group_name_ = robot_model_->getPlanningGroupName();
    
    // assign the config
    YAML::Node input_config;
    motion_planners::loadConfigFile(config_file_path, input_config);
    const YAML::Node& stomp_node = input_config["stomp"];
    const YAML::Node& debug_node = input_config["debug"];
    
    stomp_config_ = handle_stomp_config::getStompConfig(stomp_node);
    debug_config_ = handle_stomp_config::getDebugConfig(debug_node);
       
    //get planning grouup joint names.
    robot_model_->getPlanningGroupJointsName(planning_group_name_, planning_group_joints_name_);

    optimization_task_.reset(new OptimizationTask(stomp_config_, robot_model_));
    optimization_task_->stompInitialize(1,1);
    
    return true;    
}

bool StompPlanner::solve(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    optimization_task_->createPolicy();   
    
    stomp_.reset(new stomp::Stomp());
   
    stomp_->initialize(stomp_config_, optimization_task_);
    
    mkdir(debug_config_.output_dir_.c_str(), 0755);

    std::stringstream stddev_filename;
    std::stringstream num_rollouts_filename;    
    num_rollouts_filename << debug_config_.output_dir_ << "/num_rollouts.txt";
    FILE *num_rollouts_file = NULL;
    if (debug_config_.save_noisy_trajectories_)
    {
        num_rollouts_file = fopen(num_rollouts_filename.str().c_str(), "w");
    }
    
    if (debug_config_.save_noiseless_trajectories_)
    {
        std::stringstream sss;
        sss << debug_config_.output_dir_ << "/noiseless_0.txt";
        optimization_task_->policy_->writeToFile(sss.str());
        tmp_policy = *optimization_task_->policy_;
    }


    stomp::Rollout noiseless_rollout;
    double old_cost = 0.0;
    double cost_improvement = 0.0;
    
    for(int i = 0; i < stomp_config_.num_iterations_; i++)
    {
        stomp_->runSingleIteration(i);	

	stomp_->getNoiselessRollout(noiseless_rollout);
	cost_improvement = noiseless_rollout.total_cost_ - old_cost;
	old_cost = noiseless_rollout.total_cost_;

 	LOG_DEBUG_S <<"Iteration = "<<i <<". Total Cost = "<<noiseless_rollout.total_cost_<<" . Cost improvement = "<<cost_improvement
		    <<" . State costs = "<<noiseless_rollout.state_costs_;
	
	// Stop Criteria 
	// Here the noiseless_rollout.total_cost_ < 1 means there is no collision.
	// We assign a collision cost of value "1" 
	if((noiseless_rollout.total_cost_ < 1 ) && (fabs(cost_improvement) < stomp_config_.min_cost_improvement_))
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
    int end = (start + stomp_config_.num_time_steps_ -1);
    int diff = (end -start) +1;
     
    
    solution.names.resize(planning_group_joints_name_.size());
    solution.elements.resize(planning_group_joints_name_.size());

    for(int d = 0; d < stomp_config_.num_dimensions_; d++)           
    {
	solution.names.at(d) = planning_group_joints_name_.at(d);
	
	solution.elements.at(d).resize(diff+2);
	
	for( int i = 0; i <= diff+1; i++)	
	    solution.elements.at(d).at(i).position =  optimization_task_->policy_->parameters_all_[d](i+(start-1));
	
    }     

    if ((noiseless_rollout.total_cost_ < 1) && (fabs(cost_improvement) <= stomp_config_.min_cost_improvement_))
    {
	planner_status.statuscode = motion_planners::PlannerStatus::PATH_FOUND;
	return true;
    }
    else
	planner_status.statuscode = motion_planners::PlannerStatus::NO_PATH_FOUND;
    
    return false;  
}


void StompPlanner::updateInitialTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal, PlannerStatus &planner_status)
{
    optimization_task_->updateTrajectory(start, goal);
}

double StompPlanner::getMovementDeltaTime()
{
    if(optimization_task_)
	return optimization_task_->policy_->getMovementDt();
    return 0.0;
}


}// end namespace 
