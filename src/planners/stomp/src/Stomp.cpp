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

// system includes
#include <cassert>
#include <omp.h>

#include <stomp/Stomp.hpp>
#include <boost/filesystem.hpp>

namespace stomp
{

Stomp::Stomp()
: initialized_(false), policy_iteration_counter_(0)
{
}

Stomp::~Stomp()
{
}

bool Stomp::initialize(const StompConfig& config, std::shared_ptr<stomp::StompTask> task)
{
    stomp_config_ = config;
  
    stomp_task_ = task;
    
    STOMP_VERIFY(stomp_task_->getPolicy(policy_));
    STOMP_VERIFY(policy_->getNumTimeSteps(stomp_config_.num_time_steps_));
    control_cost_weight_ = stomp_task_->getControlCostWeight();

    STOMP_VERIFY(policy_->getNumDimensions(stomp_config_.num_dimensions_));
    STOMP_VERIFY(stomp_config_.num_dimensions_ == static_cast<int>(stomp_config_.noise_decay_.size()));
    STOMP_VERIFY(stomp_config_.num_dimensions_ == static_cast<int>(stomp_config_.noise_stddev_.size()));
    STOMP_VERIFY(stomp_config_.num_dimensions_ == static_cast<int>(stomp_config_.noise_min_stddev_.size()));
   

    policy_improvement_.initialize(stomp_config_.num_time_steps_, stomp_config_.min_rollouts_, stomp_config_.max_rollouts_, stomp_config_.num_rollouts_per_iteration_,
                                 policy_, stomp_config_.use_noise_adaptation_, stomp_config_.noise_min_stddev_, control_cost_weight_);

    rollout_costs_ = base::MatrixXd::Zero(stomp_config_.max_rollouts_, stomp_config_.num_time_steps_);

    policy_iteration_counter_ = 0;

    // initialize openmp
    stomp_config_.num_threads_ = omp_get_max_threads();
    if (!stomp_config_.use_openmp_)
    {
        stomp_config_.num_threads_ = 1;
        omp_set_num_threads(1);
    }
    
    tmp_rollout_cost_.resize(stomp_config_.max_rollouts_, base::VectorXd::Zero(stomp_config_.num_time_steps_));
    tmp_rollout_weighted_features_.resize(stomp_config_.max_rollouts_, base::MatrixXd::Zero(stomp_config_.num_time_steps_, 1));

    best_noiseless_cost_ = std::numeric_limits<double>::max();

    return (initialized_ = true);

}

//bool Stomp::initialize(const ros::NodeHandle& node_handle, boost::shared_ptr<stomp::Task> task)
//{
//  node_handle_ = node_handle;
//  STOMP_VERIFY(readParameters());
//
//  task_ = task;
//  STOMP_VERIFY(task_->getPolicy(policy_));
//  STOMP_VERIFY(policy_->getNumTimeSteps(num_time_steps_));
//  control_cost_weight_ = task_->getControlCostWeight();
//
//  STOMP_VERIFY(policy_->getNumDimensions(num_dimensions_));
//  //ROS_ASSERT(num_dimensions_ == static_cast<int>(noise_decay_.size()));
//  //ROS_ASSERT(num_dimensions_ == static_cast<int>(noise_stddev_.size()));
//  //ROS_ASSERT(num_dimensions_ == static_cast<int>(noise_min_stddev_.size()));
//  //    ROS_INFO("Learning policy with %i dimensions.", num_dimensions_);
//
//  policy_improvement_.initialize(num_time_steps_, min_rollouts_, max_rollouts_, num_rollouts_per_iteration_,
//                                 policy_, use_noise_adaptation_, noise_min_stddev_);
//
//  rollout_costs_ = base::MatrixXd::Zero(max_rollouts_, num_time_steps_);
//
//  policy_iteration_counter_ = 0;
//
//  // initialize openmp
//  num_threads_ = omp_get_max_threads();
//  if (!stomp_config_.use_openmp_)
//  {
//    num_threads_ = 1;
//    omp_set_num_threads(1);
//  }
//  //ROS_INFO("STOMP: using %d threads", num_threads_);
//  tmp_rollout_cost_.resize(max_rollouts_, base::VectorXd::Zero(num_time_steps_));
//  tmp_rollout_weighted_features_.resize(max_rollouts_, base::MatrixXd::Zero(num_time_steps_, 1));
//
//  best_noiseless_cost_ = std::numeric_limits<double>::max();
//
//  return (initialized_ = true);
//}

//bool Stomp::readParameters()
//{
//  STOMP_VERIFY(node_handle_.getParam("min_rollouts", min_rollouts_));
//  STOMP_VERIFY(node_handle_.getParam("max_rollouts", max_rollouts_));
//  STOMP_VERIFY(node_handle_.getParam("num_rollouts_per_iteration", num_rollouts_per_iteration_));
//  STOMP_VERIFY(readDoubleArray(node_handle_, "noise_stddev", noise_stddev_));
//  STOMP_VERIFY(readDoubleArray(node_handle_, "noise_decay", noise_decay_));
//  STOMP_VERIFY(readDoubleArray(node_handle_, "noise_min_stddev", noise_min_stddev_));
//  node_handle_.param("write_to_file", write_to_file_, true); // defaults are sometimes good!
//  node_handle_.param("use_noise_adaptation", use_noise_adaptation_, true);
//  node_handle_.param("use_openmp", stomp_config_.use_openmp_, false);
//  return true;
//}

bool Stomp::readPolicy(const int iteration_number)
{
  // check whether reading the policy from file is neccessary
  if(iteration_number == (policy_iteration_counter_))
  {
    return true;
  }
  /*    ROS_INFO("Read policy from file %s.", policy_->getFileName(iteration_number).c_str());
    STOMP_VERIFY(policy_->readFromDisc(policy_->getFileName(iteration_number)));
    STOMP_VERIFY(task_->setPolicy(policy_));
   */    return true;
}

bool Stomp::writePolicy(const int iteration_number, bool is_rollout, int rollout_id)
{
  return true;
}

void Stomp::clearReusedRollouts()
{
  policy_improvement_.clearReusedRollouts();
}

bool Stomp::doGenRollouts(int iteration_number)
{
  // compute appropriate noise values
  std::vector<double> noise;
  noise.resize(stomp_config_.num_dimensions_);
  for (int i=0; i<stomp_config_.num_dimensions_; ++i)
  {
    noise[i] = stomp_config_.noise_stddev_[i] * pow(stomp_config_.noise_decay_[i], iteration_number-1);
    //std::cout<<"NOIS = "<<noise[i]<<"  "<<stomp_config_.noise_stddev_[i] <<"  "<< pow(stomp_config_.noise_decay_[i], iteration_number-1)<<"   "<<iteration_number<< std::endl;
  }

  // get rollouts
  STOMP_VERIFY(policy_improvement_.getRollouts(rollouts_, noise));

  // filter rollouts and set them back if filtered:
  bool filtered = false;
  for (unsigned int r=0; r<rollouts_.size(); ++r)
  {     
    if (stomp_task_->filter(rollouts_[r], r, 0))
      filtered = true;
  }
  
  if (filtered)
  {
    policy_improvement_.setRollouts(rollouts_);
  }
  STOMP_VERIFY(policy_improvement_.computeProjectedNoise());

  // overwrite the rollouts with the projected versions
  policy_improvement_.getProjectedRollouts(projected_rollouts_);
  
  return true;
}

bool Stomp::doExecuteRollouts(int iteration_number)
{

  std::vector<base::VectorXd> gradients;
  #pragma omp parallel for num_threads(stomp_config_.num_threads_)

  for (int r=0; r<int(rollouts_.size()); ++r)
  {
    int thread_id = omp_get_thread_num();
    //printf("thread_id = %d\n", thread_id);
    bool validity;
    STOMP_VERIFY(stomp_task_->execute(rollouts_[r], projected_rollouts_[r], tmp_rollout_cost_[r], tmp_rollout_weighted_features_[r],
                              iteration_number, r, thread_id, false, gradients, validity));
  }
  for (int r=0; r<int(rollouts_.size()); ++r)
  {
    rollout_costs_.row(r) = tmp_rollout_cost_[r].transpose();
    //printf("Rollout %d , cost = %lf\n", r+1, tmp_rollout_cost_[r].sum());
  }
  
  //std::cout<<"---------------------------"<<std::endl;

  return true;
}

bool Stomp::doRollouts(int iteration_number)
{
  doGenRollouts(iteration_number);

  doExecuteRollouts(iteration_number);
  return true;
}

bool Stomp::doUpdate(int iteration_number)
{
  // TODO: fix this std::vector<>
  std::vector<double> all_costs;
  STOMP_VERIFY(policy_improvement_.setRolloutCosts(rollout_costs_, control_cost_weight_, all_costs));

  // improve the policy
  STOMP_VERIFY(policy_improvement_.improvePolicy(parameter_updates_));
  STOMP_VERIFY(policy_improvement_.getTimeStepWeights(time_step_weights_));
  STOMP_VERIFY(policy_->updateParameters(parameter_updates_, time_step_weights_));

  return true;
}

bool Stomp::doNoiselessRollout(int iteration_number)
{
  // get a noise-less rollout to check the cost
  std::vector<base::VectorXd> gradients;
  STOMP_VERIFY(policy_->getParameters(parameters_));
  bool validity = false;
  STOMP_VERIFY(stomp_task_->execute(parameters_, parameters_, tmp_rollout_cost_[0], tmp_rollout_weighted_features_[0], iteration_number,
                            -1, 0, false, gradients, validity));
  double total_cost;
  policy_improvement_.setNoiselessRolloutCosts(tmp_rollout_cost_[0], total_cost);
  
  //printf("Noiseless cost = %lf", total_cost);
  if (total_cost < best_noiseless_cost_)
  {
    best_noiseless_parameters_ = parameters_;
    best_noiseless_cost_ = total_cost;
  }
  last_noiseless_rollout_valid_ = validity;
  return true;
}

bool Stomp::runSingleIteration(const int iteration_number)
{
  STOMP_VERIFY(initialized_);
  policy_iteration_counter_++;


  if (debug_config_.write_to_file_)
  {
    // load new policy if neccessary
    STOMP_VERIFY(readPolicy(iteration_number));
  }

  STOMP_VERIFY(doRollouts(iteration_number));

  STOMP_VERIFY(doUpdate(iteration_number));

  STOMP_VERIFY(doNoiselessRollout(iteration_number));


//   if (debug_config_.write_to_file_)
//   {
//     // store updated policy to disc
//     //STOMP_VERIFY(writePolicy(iteration_number));
//     //STOMP_VERIFY(writePolicyImprovementStatistics(stats_msg));
//   }

  return true;
}

void Stomp::getAllRollouts(std::vector<Rollout>& rollouts)
{
  policy_improvement_.getAllRollouts(rollouts);
}

void Stomp::getNoiselessRollout(Rollout& rollout)
{
  policy_improvement_.getNoiselessRollout(rollout);
}


double Stomp::getNoiselessRolloutTotalCost()
{
  return policy_improvement_.getNoiselessRolloutTotalCost();
}

void Stomp::getAdaptedStddevs(std::vector<double>& stddevs)
{
  policy_improvement_.getAdaptedStddevs(stddevs);
}

void Stomp::getBestNoiselessParameters(std::vector<base::VectorXd>& parameters, double& cost)
{
  parameters = best_noiseless_parameters_;
  cost = best_noiseless_cost_;
}

bool Stomp::runUntilValid(int max_iterations, int iterations_after_collision_free)
{
  int collision_free_iterations = 0;
  bool success = false;
  for (int i=0; i<max_iterations; ++i)
  {
    runSingleIteration(i);
    stomp_task_->onEveryIteration();
    if (last_noiseless_rollout_valid_)
    {
      success = true;
      collision_free_iterations++;
    }
//    else
//    {
//      collision_free_iterations = 0;
//    }
    if (collision_free_iterations>=iterations_after_collision_free)
    {
      break;
    }
  }

  return success;
}

void Stomp::setCostCumulation(bool use_cumulative_costs)
{
  policy_improvement_.setCostCumulation(use_cumulative_costs);
}

void Stomp::resetAdaptiveNoise()
{
  policy_improvement_.resetAdaptiveNoise();
}

}
