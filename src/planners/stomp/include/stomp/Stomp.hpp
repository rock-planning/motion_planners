/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */
/** \modified by Sankaranarayanan Natarajan  */

#ifndef STOMP_HPP_
#define STOMP_HPP_

#include <boost/shared_ptr.hpp>

#include <stomp/StompConfig.hpp>
#include <stomp/CovariantMovementPrimitive.hpp>
#include <stomp/StompTask.hpp>
#include <stomp/PolicyImprovement.hpp>

namespace stomp
{

class Stomp
{
public:
    Stomp();
    virtual ~Stomp();

    // task must already be initialized at this point.
    bool initialize(const StompConfig& config, std::shared_ptr<StompTask> task);

    bool runSingleIteration(int iteration_number);
    void clearReusedRollouts();

    bool doGenRollouts(int iteration_number);
    bool doExecuteRollouts(int iteration_number);
    bool doRollouts(int iteration_number);
    bool doUpdate(int iteration_number);
    bool doNoiselessRollout(int iteration_number);

    void getAllRollouts(std::vector<Rollout>& rollouts);
    void getNoiselessRollout(Rollout& rollout);
    double getNoiselessRolloutTotalCost();
    void getAdaptedStddevs(std::vector<double>& stddevs);
    void getBestNoiselessParameters(std::vector<base::VectorXd>& parameters, double& cost);

    bool runUntilValid(int max_iterations, int iterations_after_collision_free);
    void setCostCumulation(bool use_cumulative_costs);

    void resetAdaptiveNoise();

private:

    bool initialized_;

    StompConfig stomp_config_;
    DebugConfig debug_config_;
//    int num_threads_;
//
//    int min_rollouts_;
//    int max_rollouts_;
//    int num_rollouts_per_iteration_;
//    int num_time_steps_;
//    int num_dimensions_;
//
//    bool write_to_file_;
//    bool use_noise_adaptation_;
//    bool use_openmp_;

    std::shared_ptr<StompTask> stomp_task_;
    boost::shared_ptr<CovariantMovementPrimitive> policy_;

    PolicyImprovement policy_improvement_;

    std::vector<base::VectorXd> best_noiseless_parameters_;
    double best_noiseless_cost_;

    bool last_noiseless_rollout_valid_;

    std::vector<std::vector<base::VectorXd> > rollouts_; /**< [num_rollouts][num_dimensions] num_parameters */
    std::vector<std::vector<base::VectorXd> > projected_rollouts_;
    std::vector<base::MatrixXd> parameter_updates_;
    std::vector<base::VectorXd> parameters_;
    std::vector<base::VectorXd> time_step_weights_;
    base::MatrixXd rollout_costs_;
    //std::vector<double> noise_stddev_;
    //std::vector<double> noise_decay_;
    //std::vector<double> noise_min_stddev_;
    double control_cost_weight_;

    // temporary variables
    std::vector<base::VectorXd> tmp_rollout_cost_;
    std::vector<base::MatrixXd> tmp_rollout_weighted_features_;

    bool readParameters();

    int policy_iteration_counter_;
    bool readPolicy(const int iteration_number);
    bool writePolicy(const int iteration_number, bool is_rollout = false, int rollout_id = 0);

    //bool writePolicyImprovementStatistics(const policy_improvement_loop::PolicyImprovementStatistics& stats_msg);

};

}

#endif /* POLICY_IMPROVEMENT_LOOP_H_ */
