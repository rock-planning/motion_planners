#ifndef STOMP_CONFIG_HPP_
#define STOMP_CONFIG_HPP_

#include <string>
#include <vector>

namespace stomp
{

struct DebugConfig
{
    std::string output_dir_;
    bool save_noisy_trajectories_;
    bool save_noiseless_trajectories_;
    bool save_cost_function_;
    bool write_to_file_;
};

struct StompConfig
{

    int num_threads_;                                     
    int min_rollouts_;
    int max_rollouts_;
    int num_rollouts_per_iteration_;
    int num_time_steps_;
    int num_dimensions_;
    int num_iterations_;
    
    double movement_duration_;
    double control_cost_weight_;
    double delay_per_iteration_;
    double resolution_;

    std::vector<double> noise_stddev_;
    std::vector<double> noise_decay_;
    std::vector<double> noise_min_stddev_;
                                     
    bool use_noise_adaptation_;
    bool use_openmp_;
};


}

#endif
