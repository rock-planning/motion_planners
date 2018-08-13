/*
 * stomp_2d_test.h
 *
 *  Created on: Feb 2, 2012
 *      Author: kalakris
 */

#ifndef STOMP_2D_TEST_H_
#define STOMP_2D_TEST_H_

#include <stomp/Stomp.hpp>
#include <stomp/StompTask.hpp>
//#include <std/enable_shared_from_this.hpp>
//#include <stomp/HandleConfig.hpp>

namespace stomp
{

struct Obstacle
{
  std::vector<double> center_;
  std::vector<double> radius_;
  bool boolean_;
};

class Stomp2DTest: public StompTask, public std::enable_shared_from_this<Stomp2DTest>
{
public:

  Stomp2DTest()
  {
  }

  int run(YAML::Node &input_config, YAML::Node &obstacles_config);

  // functions inherited from Task:

  /**
   * Initialize the task for a given number of threads.
   * @param num_threads Number of threads for multi-threading
   * @return
   */
  virtual bool stompInitialize(int num_threads, int num_rollouts);

  /**
   * Executes the task for the given policy parameters, and returns the costs per timestep
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
   * @param weighted_feature_values num_time_steps x num_features matrix of weighted feature values per time step
   * @return
   */
  virtual bool execute(std::vector<Eigen::VectorXd>& parameters,
                       std::vector<Eigen::VectorXd>& projected_parameters,
                       Eigen::VectorXd& costs,
                       Eigen::MatrixXd& weighted_feature_values,
                       const int iteration_number,
                       const int rollout_number,
                       int thread_id,
                       bool compute_gradients,
                       std::vector<Eigen::VectorXd>& gradients,
                       bool& validity);

  virtual bool filter(std::vector<Eigen::VectorXd>& parameters, int rollout_id, int thread_id);

  /**
   * Get the Policy object of this Task
   * @param policy
   * @return
   */
  virtual bool getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy);

  /**
   * Sets the Policy object of this Task
   * @param policy
   * @return
   */
  virtual bool setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy);

  /**
   * Gets the weight of the control cost
   * @param control_cost_weight
   * @return
   */
  virtual double getControlCostWeight();

private:
    std::shared_ptr<stomp::Stomp> stomp_;
    boost::shared_ptr<stomp::CovariantMovementPrimitive> policy_;

    stomp::StompConfig stomp_config;
    stomp::DebugConfig debug_config;

    //int num_iterations_;
    //int num_time_steps_;
    //double movement_duration_;
    //double control_cost_weight_;
    //double delay_per_iteration_;
    //int num_dimensions_;
    double movement_dt_;
    //
    //std::string output_dir_;
    //bool save_noisy_trajectories_;
    //bool save_noiseless_trajectories_;
    //bool save_cost_function_;

    //double resolution_;

  double cost_viz_scaling_const_;
  double cost_viz_scaling_factor_;
  std::vector<Obstacle> obstacles_;

  Eigen::MatrixXd vel_diff_matrix_;
  Eigen::MatrixXd acc_diff_matrix_;

  std::vector<Eigen::VectorXd> initial_trajectory_;

  void readParameters(YAML::Node &input_config);
  void readObstacles(YAML::Node &obstacles_config);
  void writeCostFunction();
  void visualizeCostFunction();
  void visualizeTrajectory(Rollout& rollout, bool noiseless, int id);

  double evaluateMapCost(double x, double y) const;
  void evaluateMapGradients(double x, double y, double& gx, double& gy) const;
  double evaluateCost(double x, double y, double vx, double vy) const;
  double evaluateCostWithGradients(double x, double y, double vx, double vy,
                                  bool compute_gradients,
                                  double ax, double ay, double& gx, double& gy) const;
  double evaluateCostPath(double x1, double y1, double x2, double y2, double vx, double vy) const;
  double evaluateCostPathWithGradients(double x1, double y1, double x2, double y2, double vx, double vy,
                                       bool compute_gradients,
                                       double ax, double ay, double& gx, double& gy) const;

};

} /* namespace stomp */
#endif /* STOMP_2D_TEST_H_ */
