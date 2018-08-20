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
    }
    
    stomp::CovariantMovementPrimitive tmp_policy = *optimization_task_->policy_;
   
    auto start_time = std::chrono::high_resolution_clock::now();
    auto finish_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_time - start_time;

    auto start_iter_time = std::chrono::high_resolution_clock::now();
    int iter_ct = 0;
    stomp::Rollout noiseless_rollout;
    noiseless_rollout.total_cost_ = 100.0;
    while ((iter_ct < stomp_config_.num_iterations_) && (noiseless_rollout.total_cost_ > 0.1))
    {	
	//std::cout<<"Iteration = "<<i<<std::endl;
        //start_time = std::chrono::high_resolution_clock::now();
        stomp_->runSingleIteration(iter_ct);
	iter_ct ++;
	//finish_time = std::chrono::high_resolution_clock::now();
	//elapsed = finish_time - start_time;
	//std::cout << "Elapsed time Iter: " << elapsed.count() << " s\n";
	              
	
        stomp_->getNoiselessRollout(noiseless_rollout);
	std::cout<<iter_ct <<" Cost ="<<noiseless_rollout.total_cost_<<std::endl;

        if (debug_config_.save_noisy_trajectories_)
        {
	    std::vector<stomp::Rollout> rollouts;
	    stomp_->getAllRollouts(rollouts);
          fprintf(num_rollouts_file, "%d\n", int(rollouts.size()));
          for (unsigned int j=0; j<rollouts.size(); ++j)
          {
            std::stringstream ss2;
            ss2 << debug_config_.output_dir_ << "/noisy_" << iter_ct << "_" << j << ".txt";
            //tmp_policy.setParameters(rollouts[j].parameters_noise_projected_);
            tmp_policy.setParameters(rollouts[j].parameters_noise_);
            tmp_policy.writeToFile(ss2.str());
          }
        }
        
        if (debug_config_.save_noiseless_trajectories_)
        {
          std::stringstream ss;
          ss << debug_config_.output_dir_ << "/noiseless_" << iter_ct << ".txt";
          optimization_task_->policy_->writeToFile(ss.str());          
        }
    }
    
    auto finish_iter_time = std::chrono::high_resolution_clock::now();  
    elapsed = finish_iter_time - start_iter_time;
    std::cout << "Elapsed time solve tim: " << elapsed.count() << " s\n";
	
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

  if (debug_config_.save_noisy_trajectories_)
    fclose(num_rollouts_file);
  stomp_.reset();
  
}


void StompPlanner::updateInitialTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal, PlannerStatus &planner_status)
{
    optimization_task_->updateTrajectory(start, goal);
}

}// end namespace 
