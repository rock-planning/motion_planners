#include <wrapper/stomp/HandleStompConfig.hpp>

namespace handle_stomp_config
{


stomp::StompConfig getStompConfig(const YAML::Node &yaml_data)
{

    stomp::StompConfig config;

    config.num_threads_                 = motion_planners::getValue<int>(yaml_data, "num_thread_");
    config.min_rollouts_                = motion_planners::getValue<int>(yaml_data, "min_rollouts_");
    config.max_rollouts_                = motion_planners::getValue<int>(yaml_data, "max_rollouts_");
    config.num_rollouts_per_iteration_  = motion_planners::getValue<int>(yaml_data, "num_rollouts_per_iteration_");
    config.num_time_steps_              = motion_planners::getValue<int>(yaml_data, "num_time_steps_");
    config.num_dimensions_              = motion_planners::getValue<int>(yaml_data, "num_dimensions_");
    config.num_iterations_              = motion_planners::getValue<int>(yaml_data, "num_iterations_"); 


    const YAML::Node &noise_std_node    = yaml_data["noise_stddev_"];
    config.noise_stddev_.resize(noise_std_node.size());
    for(std::size_t i = 0; i < noise_std_node.size(); i++)
        config.noise_stddev_.at(i) = noise_std_node[i].as<double>();

    
    const YAML::Node &noise_decay_node  = yaml_data["noise_decay_"];
    config.noise_decay_.resize(noise_decay_node.size());
    for(std::size_t i = 0; i < noise_decay_node.size(); i++)
        config.noise_decay_.at(i) = noise_decay_node[i].as<double>();

    
    const YAML::Node &noise_mstd_node   = yaml_data["noise_min_stddev_"];
    config.noise_min_stddev_.resize(noise_mstd_node.size());
    for(std::size_t i = 0; i < noise_mstd_node.size(); i++)
        config.noise_min_stddev_.at(i) = noise_mstd_node[i].as<double>();

   
    config.movement_duration_           = motion_planners::getValue<double>(yaml_data, "movement_duration_"); 
    config.control_cost_weight_         = motion_planners::getValue<double>(yaml_data, "control_cost_weight_"); 
    config.delay_per_iteration_         = motion_planners::getValue<double>(yaml_data, "delay_per_iteration_"); 
    config.resolution_                  = motion_planners::getValue<double>(yaml_data, "resolution_"); 

    config.use_noise_adaptation_        = motion_planners::getValue<bool>(yaml_data, "use_noise_adaptation_");
    config.use_openmp_                  = motion_planners::getValue<bool>(yaml_data, "use_openmp_");

    return config;
}

stomp::DebugConfig getDebugConfig(const YAML::Node &yaml_data)
{

    stomp::DebugConfig config;

    config.output_dir_                  = motion_planners::getValue<std::string>(yaml_data, "output_dir_");
    config.save_noisy_trajectories_     = motion_planners::getValue<bool>(yaml_data, "save_noisy_trajectories_");
    config.save_noiseless_trajectories_ = motion_planners::getValue<bool>(yaml_data, "save_noiseless_trajectories_");
    config.save_cost_function_          = motion_planners::getValue<bool>(yaml_data, "save_cost_function_");
    config.write_to_file_               = motion_planners::getValue<bool>(yaml_data, "write_to_file_");

    return config;
}

}
