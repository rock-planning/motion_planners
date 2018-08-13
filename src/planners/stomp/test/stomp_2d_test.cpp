/*
 * stomp_2d_test.cpp
 *
 *  Created on: Feb 2, 2012
 *      Author: kalakris
 */

#include "stomp_2d_test.h"
#include <sstream>
#include <cstdio>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <time.h>
#include <stomp/StompUtils.hpp>

namespace stomp
{

int Stomp2DTest::run(YAML::Node &input_config, YAML::Node &obstacles_config)
{

    srand(time(NULL));
    readObstacles(obstacles_config);
    readParameters(input_config);
    mkdir(debug_config.output_dir_.c_str(), 0755);

    std::stringstream stddev_filename, cost_filename;
    std::stringstream num_rollouts_filename;
    stddev_filename << debug_config.output_dir_ << "/stddevs.txt";
    cost_filename << debug_config.output_dir_ << "/costs.txt";
    num_rollouts_filename << debug_config.output_dir_ << "/num_rollouts.txt";
  
    FILE *stddev_file = fopen(stddev_filename.str().c_str(), "w");
    FILE *cost_file = fopen(cost_filename.str().c_str(), "w");
    FILE *num_rollouts_file = NULL;
    if (debug_config.save_noisy_trajectories_)
    {
        num_rollouts_file = fopen(num_rollouts_filename.str().c_str(), "w");
    }

    std::vector<Eigen::MatrixXd> derivative_costs;
    std::vector<Eigen::VectorXd> initial_trajectory;
    derivative_costs.resize(stomp_config.num_dimensions_, Eigen::MatrixXd::Zero(stomp_config.num_time_steps_ + 2*TRAJECTORY_PADDING, NUM_DIFF_RULES));
    initial_trajectory.resize(stomp_config.num_dimensions_, Eigen::VectorXd::Zero(stomp_config.num_time_steps_ + 2*TRAJECTORY_PADDING));
//     std::cout<<"Init deri traj size = "<<derivative_costs[0].size()<<std::endl<<derivative_costs[0]<<std::endl;
    std::cout<<"Init traj size = "<<initial_trajectory[0].size()<<std::endl<<initial_trajectory[0]<<std::endl;
    for (int d=0; d<stomp_config.num_dimensions_; ++d)
    {
        derivative_costs[d].col(STOMP_ACCELERATION) = Eigen::VectorXd::Ones(stomp_config.num_time_steps_ + 2*TRAJECTORY_PADDING);
        initial_trajectory[d].head(TRAJECTORY_PADDING) = 0.00*Eigen::VectorXd::Ones(TRAJECTORY_PADDING);
        initial_trajectory[d].tail(TRAJECTORY_PADDING) = 0.99*Eigen::VectorXd::Ones(TRAJECTORY_PADDING);

std::cout<<"After Init deri traj size = "<<derivative_costs[d].size()<<std::endl<<derivative_costs[0]<<std::endl;
std::cout<<"After Init traj size = "<<initial_trajectory[d].size()<<std::endl<<initial_trajectory[0]<<std::endl;
    }
    

    std::cout<<"Initialsing policy"<<std::endl;
    policy_.reset(new CovariantMovementPrimitive());
    policy_->initialize(stomp_config.num_time_steps_,
                      stomp_config.num_dimensions_,
                      stomp_config.movement_duration_,
                      derivative_costs,
                      initial_trajectory);
    policy_->setToMinControlCost();
    policy_->getParametersAll(initial_trajectory_);
    vel_diff_matrix_ = policy_->getDifferentiationMatrix(stomp::STOMP_VELOCITY);
    acc_diff_matrix_ = policy_->getDifferentiationMatrix(stomp::STOMP_ACCELERATION);
    std::cout<<"Vel dif matr = "<<std::endl<<vel_diff_matrix_<<std::endl;
    std::cout<<"Acc dif matr = "<<std::endl<<acc_diff_matrix_<<std::endl;
    std::cout<<"Init ="<<std::endl<<initial_trajectory[0]<<std::endl<<"kkkkk"<<initial_trajectory[1]<<std::endl;
    
    std::cout<<"VEEEE "<<std::endl<<vel_diff_matrix_*initial_trajectory[0]<<std::endl;
    std::cout<<"VEEEE "<<std::endl<<vel_diff_matrix_*initial_trajectory[1]<<std::endl;
    std::cout<<"Acc "<<std::endl<<acc_diff_matrix_*initial_trajectory[0]<<std::endl;
    
    movement_dt_ = policy_->getMovementDt();

    
    if (debug_config.save_cost_function_)
    {
        std::cout<<"Writing cost function"<<std::endl;
        writeCostFunction();
    }

    std::cout<<"Initialising STOMP"<<std::endl;
    stomp_.reset(new stomp::Stomp());
    // we already in readparameter
    //stomp::StompConfig stomp_config = handel_config::getStompConfig(input_config);
    stomp_->initialize(stomp_config, shared_from_this());
 
    if (debug_config.save_noiseless_trajectories_)
    {
        std::stringstream sss;
        sss << debug_config.output_dir_ << "/noiseless_0.txt";
        policy_->writeToFile(sss.str());
    }

    CovariantMovementPrimitive tmp_policy = *policy_;

//  ros::Time prev_iter_stamp = ros::Time::now();

    for (int i=1; i<=stomp_config.num_iterations_; ++i)
    {
        std::vector<Rollout> rollouts;
        Rollout noiseless_rollout;
        //std::cout<<"Running iteration = "<<i<<std::endl;
        stomp_->runSingleIteration(i);
        //std::cout<<"Getting rollouts"<<std::endl;
        stomp_->getAllRollouts(rollouts);
        //std::cout<<"Getting noiselessrollouts"<<std::endl;
        stomp_->getNoiselessRollout(noiseless_rollout);

        std::vector<double> stddevs;
        stomp_->getAdaptedStddevs(stddevs);
        if (debug_config.save_noiseless_trajectories_)
        {
          for (unsigned int d=0; d<stddevs.size(); ++d)
          {
            fprintf(stddev_file, "%f\t", stddevs[d]);
          }
          fprintf(stddev_file, "\n");
        }


        if (debug_config.save_noiseless_trajectories_)
        {
          std::stringstream ss;
          ss << debug_config.output_dir_ << "/noiseless_" << i << ".txt";
          policy_->writeToFile(ss.str());
          fprintf(cost_file, "%f\n", noiseless_rollout.total_cost_);
        }

        if (debug_config.save_noisy_trajectories_)
        {
          fprintf(num_rollouts_file, "%d\n", int(rollouts.size()));
          for (unsigned int j=0; j<rollouts.size(); ++j)
          {
            std::stringstream ss2;
            ss2 << debug_config.output_dir_ << "/noisy_" << i << "_" << j << ".txt";
            //tmp_policy.setParameters(rollouts[j].parameters_noise_projected_);
            tmp_policy.setParameters(rollouts[j].parameters_noise_);
            tmp_policy.writeToFile(ss2.str());
          }
        }
        printf("%f\n", noiseless_rollout.total_cost_);

  }

  fclose(stddev_file);
  fclose(cost_file);
  if (debug_config.save_noisy_trajectories_)
    fclose(num_rollouts_file);

  stomp_.reset();
  policy_.reset();

  return 0;
}

bool Stomp2DTest::stompInitialize(int num_threads, int num_rollouts)
{
  return true;
}

void Stomp2DTest::writeCostFunction()
{
  std::stringstream ss;
  ss << debug_config.output_dir_ << "/cost_function.txt";
  int num_x = lrint(1.0 / stomp_config.resolution_) + 1;
  int num_y = lrint(1.0 / stomp_config.resolution_) + 1;

  FILE *f = fopen(ss.str().c_str(), "w");
  fprintf(f, "%d\t%d\n", num_x, num_y);
  for (int i=0; i<num_x; ++i)
  {
    double x = i*stomp_config.resolution_;
    for (int j=0; j<num_y; ++j)
    {
      double y = j*stomp_config.resolution_;
      double cost = evaluateMapCost(x, y);
      fprintf(f, "%lf\t%lf\t%lf\n", x, y, cost);
    }
  }
  fclose(f);
}

void Stomp2DTest::readObstacles(YAML::Node &obstacles_config)
{
    std::cout<<"obst = "<<obstacles_config.size()<<std::endl;  
    
    for(unsigned int i = 0; i < obstacles_config.size(); i++)
    {
        std::vector<double> center(2);
        center[0] = obstacles_config[i]["center"][0].as<double>();
        center[1] = obstacles_config[i]["center"][1].as<double>();

        std::vector<double> radius(2);
        radius[0] = obstacles_config[i]["radius"][0].as<double>();
        radius[1] = obstacles_config[i]["radius"][1].as<double>();

        bool boolean;
        boolean = obstacles_config[i]["boolean"].as<bool>();
        std::cout<<center[0]<<"  "<<center[1]<<"  "<<radius[0]<<"  "<<radius[1]<<std::endl;

        Obstacle o;
        o.center_ = center;
        o.radius_ = radius;
        o.boolean_ = boolean;
        obstacles_.push_back(o);
    }        
}


void Stomp2DTest::readParameters(YAML::Node &input_config)
{


    const YAML::Node& debug_node = input_config["debug"];
    const YAML::Node& stomp_node = input_config["stomp"];

    //std::cout<<"Size of stomp config" <<stomp_node.size()<<std::endl;
    debug_config = handle_config::getDebugConfig(debug_node);
    stomp_config = handle_config::getStompConfig(stomp_node);

    std::cout<<stomp_config.max_rollouts_<<std::endl;


/*  // WARNING, TODO: no error checking here!!!
  obstacles_.clear();
  XmlRpc::XmlRpcValue obstacles_xml;
  STOMP_VERIFY(node_handle_.getParam("cost_function", obstacles_xml));
  for (int i=0; i<obstacles_xml.size(); ++i)
  {
    Obstacle o;
    STOMP_VERIFY(getParam(obstacles_xml[i], "center", o.center_));
    STOMP_VERIFY(getParam(obstacles_xml[i], "radius", o.radius_));
    STOMP_VERIFY(getParam(obstacles_xml[i], "boolean", o.boolean_));
    obstacles_.push_back(o);
  }

  STOMP_VERIFY(node_handle_.getParam("num_iterations", num_iterations_));

  STOMP_VERIFY(node_handle_.getParam("movement_duration", movement_duration_));
  STOMP_VERIFY(node_handle_.getParam("control_cost_weight", control_cost_weight_));
  STOMP_VERIFY(node_handle_.getParam("output_dir", debug_config.output_dir_));
  STOMP_VERIFY(node_handle_.getParam("use_chomp", use_chomp_));
  STOMP_VERIFY(node_handle_.getParam("save_noisy_trajectories", debug_config.save_noisy_trajectories_));
  STOMP_VERIFY(node_handle_.getParam("save_noiseless_trajectories", debug_config.save_noiseless_trajectories_));
  STOMP_VERIFY(node_handle_.getParam("save_cost_function", debug_config.save_cost_function_));
  STOMP_VERIFY(node_handle_.getParam("publish_to_rviz", publish_to_rviz_));
  STOMP_VERIFY(node_handle_.getParam("delay_per_iteration", delay_per_iteration_));



  STOMP_VERIFY(node_handle_.getParam("num_time_steps", num_time_steps_));
*/
}

bool Stomp2DTest::execute(std::vector<Eigen::VectorXd>& parameters,
                          std::vector<Eigen::VectorXd>& projected_parameters,
                          Eigen::VectorXd& costs,
                     Eigen::MatrixXd& weighted_feature_values,
                     const int iteration_number,
                     const int rollout_number,
                     int thread_id,
                     bool compute_gradients,
                     std::vector<Eigen::VectorXd>& gradients,
                     bool& validity)
{
  costs = Eigen::VectorXd::Zero(stomp_config.num_time_steps_);
  //weighted_feature_values = Eigen::MatrixXd::Zero(num_time_steps_, 1);
  Eigen::MatrixXd proj_pos(stomp_config.num_dimensions_, stomp_config.num_time_steps_ + 2*TRAJECTORY_PADDING);
  Eigen::MatrixXd pos(stomp_config.num_dimensions_, stomp_config.num_time_steps_ + 2*TRAJECTORY_PADDING);
  Eigen::MatrixXd vel(stomp_config.num_dimensions_, stomp_config.num_time_steps_ + 2*TRAJECTORY_PADDING);
  Eigen::MatrixXd acc(stomp_config.num_dimensions_, stomp_config.num_time_steps_ + 2*TRAJECTORY_PADDING);

  for (int d=0; d<stomp_config.num_dimensions_; ++d)
  {
    proj_pos.row(d) = initial_trajectory_[d];
    pos.row(d) = initial_trajectory_[d];
    //printf("lvalue size = %d, %d, rvalue size = %d, %d", 1, stomp_config.num_time_steps_, projected_parameters[d].rows(), projected_parameters[d].cols());
    proj_pos.block(d, TRAJECTORY_PADDING, 1, stomp_config.num_time_steps_) = projected_parameters[d].transpose();
    pos.block(d, TRAJECTORY_PADDING, 1, stomp_config.num_time_steps_) = parameters[d].transpose();
  }
  if (compute_gradients)
  {
    gradients.resize(stomp_config.num_dimensions_, Eigen::VectorXd::Zero(stomp_config.num_time_steps_));
  }

  vel = (vel_diff_matrix_ * pos.transpose()).transpose();
  if (compute_gradients)
  {
    acc = (acc_diff_matrix_ * pos.transpose()).transpose();
   }

  double px = 0.01;
  double py = 0.01;
  for (int t=TRAJECTORY_PADDING; t<TRAJECTORY_PADDING+stomp_config.num_time_steps_; ++t)
  {
    double x = pos(0,t);
    double y = pos(1,t);
    double cost = 0.0;
    if (compute_gradients)
    {
      cost = evaluateCostPathWithGradients(px, py, x, y, vel(0,t), vel(1,t), true,
                                           acc(0,t), acc(1,t), gradients[0](t-TRAJECTORY_PADDING), gradients[1](t-TRAJECTORY_PADDING));
    }
    else
    {
      cost = evaluateCostPath(px, py, x, y, vel(0,t), vel(1,t));
    }
    costs(t-TRAJECTORY_PADDING) = cost;
    px = x;
    py = y;
  }
  validity = true;
  return true;
}

double Stomp2DTest::evaluateCost(double x, double y, double vx, double vy) const
{
  double ax=0.0, ay=0.0, gx=0.0, gy=0.0;
  return evaluateCostWithGradients(x, y, vx, vy, false, ax, ay, gx, gy);
}

double Stomp2DTest::evaluateMapCost(double x, double y) const
{
  double cost = 0.0;
  for (unsigned int o=0; o<obstacles_.size(); ++o)
  {
    double dx = (x - obstacles_[o].center_[0])/obstacles_[o].radius_[0];
    double dy = (y - obstacles_[o].center_[1])/obstacles_[o].radius_[1];

    double dist = dx * dx + dy * dy;

    if (obstacles_[o].boolean_)
    {
      if (dist < 1.0)
      {
        //cost += 1.0;
        if (cost < 1.0)
          cost = 1.0;
      }
    }
    else
    {
      if (dist < 1.0)
      {
        //cost += 1.0 - dist;
        if (cost < 1.0-dist)
          cost = 1.0-dist;
      }
    }
  }

  // joint limits
  const double joint_limit_cost = 100.0;
  if (x < 0.0)
  {
    cost += joint_limit_cost * -x;
  }
  if (x > 1.0)
  {
    cost += joint_limit_cost * (x - 1.0);
  }
  if (y < 0.0)
  {
    cost += joint_limit_cost * -y;
  }
  if (y > 1.0)
  {
    cost += joint_limit_cost * (y - 1.0);
  }
  return cost;
}

void Stomp2DTest::evaluateMapGradients(double x, double y, double& gx, double& gy) const
{
  gx = (evaluateMapCost(x+stomp_config.resolution_, y) - evaluateMapCost(x-stomp_config.resolution_, y)) / (2*stomp_config.resolution_);
  gy = (evaluateMapCost(x, y+stomp_config.resolution_) - evaluateMapCost(x, y-stomp_config.resolution_)) / (2*stomp_config.resolution_);
}

double Stomp2DTest::evaluateCostWithGradients(double x, double y, double vx, double vy,
                                bool compute_gradients,
                                double ax, double ay, double& gx, double& gy) const
{
  double cost = evaluateMapCost(x,y) * movement_dt_;
  double vel_mag = sqrt(vx*vx + vy*vy);

  if (compute_gradients)
  {
    double map_gx=0.0, map_gy=0.0;
    evaluateMapGradients(x, y, map_gx, map_gy);

    map_gx *= movement_dt_;
    map_gy *= movement_dt_;

    Eigen::Vector2d vel;
    Eigen::Vector2d norm_vel;
    vel(0) = vx;
    vel(1) = vy;
    norm_vel = vel.normalized();
    Eigen::Matrix2d orth_proj = Eigen::Matrix2d::Identity() - norm_vel*norm_vel.transpose();
    Eigen::Vector2d acc;
    acc(0) = ax;
    acc(1) = ay;
    Eigen::Vector2d curvature = (1.0/vel.squaredNorm()) * orth_proj * acc;
    Eigen::Vector2d grad;
    grad(0) = map_gx;
    grad(1) = map_gy;
    Eigen::Vector2d new_grad = vel_mag * (orth_proj*grad - cost*curvature);
    gx = new_grad(0);
    gy = new_grad(1);
  }

  return cost*vel_mag;
}

double Stomp2DTest::evaluateCostPath(double x1, double y1, double x2, double y2, double vx, double vy) const
{
  double ax = 0.0, ay = 0.0, gx = 0.0, gy = 0.0;
  return evaluateCostPathWithGradients(x1, y1, x2, y2, vx, vy, false, ax, ay, gx, gy);
}

double Stomp2DTest::evaluateCostPathWithGradients(double x1, double y1, double x2, double y2, double vx, double vy,
                                     bool compute_gradients,
                                     double ax, double ay, double& gx, double& gy) const
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  double dist = sqrt(dx*dx + dy*dy);
  int num_samples = ceil(dist / stomp_config.resolution_);
  if (compute_gradients)
  {
    gx = 0.0;
    gy = 0.0;
  }
  if (num_samples == 0)
    return 0.0;
  if (num_samples > 20)
    num_samples = 20;
  double cost = 0.0;
  for (int i=0; i<num_samples; ++i) // leave out the last one to avoid double counting
  {
    double d = (double(i) / double(num_samples));
    double x = x1 + d*dx;
    double y = y1 + d*dy;
    double temp_gx=0.0, temp_gy=0.0;
    cost += evaluateCostWithGradients(x, y, vx, vy, compute_gradients, ax, ay, temp_gx, temp_gy);
    gx += temp_gx;
    gy += temp_gy;
  }
  cost /= num_samples;
  if (compute_gradients)
  {
    gx /= num_samples;
    gy /= num_samples;
  }
  return cost;
}

bool Stomp2DTest::filter(std::vector<Eigen::VectorXd>& parameters, int rollout_id, int thread_id)
{
  return false;
  bool filtered = false;
  for (unsigned int d=0; d<parameters.size(); ++d)
  {
    for (int t=0; t<stomp_config.num_time_steps_; ++t)
    {
      if (parameters[d](t) < 0.0)
      {
        parameters[d](t) = 0.0;
        filtered = true;
      }
      if (parameters[d](t) > 1.0)
      {
        parameters[d](t) = 1.0;
        filtered = true;
      }
    }
  }
  return filtered;
}

bool Stomp2DTest::getPolicy(boost::shared_ptr<stomp::CovariantMovementPrimitive>& policy)
{
  policy = policy_;
  return true;
}

bool Stomp2DTest::setPolicy(const boost::shared_ptr<stomp::CovariantMovementPrimitive> policy)
{
  policy_ = policy;
  return true;
}

double Stomp2DTest::getControlCostWeight()
{
  return stomp_config.control_cost_weight_;
}

/*
void Stomp2DTest::visualizeCostFunction()
{
  visualization_msgs::Marker marker;

  int num_x = lrint(1.0 / resolution_) + 1;
  int num_y = lrint(1.0 / resolution_) + 1;

  marker.id = 0;
  marker.ns = "cost";
  marker.header.frame_id = "BASE";
  marker.header.stamp = ros::Time::now();
  marker.type = marker.CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = resolution_*2.0;
  marker.scale.y = resolution_*2.0;
  marker.scale.z = resolution_*2.0;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.points.reserve(num_x*num_y);
  marker.colors.reserve(num_x*num_y);

  std_msgs::ColorRGBA color;
  geometry_msgs::Point point;

  double min_cost = std::numeric_limits<double>::max();
  double max_cost = std::numeric_limits<double>::min();
  for (int i=0; i<num_x; ++i)
  {
    double x = i*resolution_;
    for (int j=0; j<num_y; ++j)
    {
      double y = j*resolution_;
      double cost = evaluateMapCost(x, y);
      if (cost > max_cost)
        max_cost = cost;
      if (cost < min_cost)
        min_cost = cost;
      point.x = x;
      point.y = y;
      point.z = cost; // temp storage
      marker.points.push_back(point);
    }
  }

  // now loop and set colors based on the scaling
  for (size_t i=0; i<marker.points.size(); ++i)
  {
    double cost = marker.points[i].z;
    double scaled_cost = (cost - min_cost)/(max_cost - min_cost);
    color.r = scaled_cost;
    color.b = 0.5 * (1.0 - scaled_cost);
    color.g = 0.0;
    color.a = 1.0;
    marker.colors.push_back(color);
    marker.points[i].z = 0.1 * scaled_cost;

    // interchange axes x and z
    double x = marker.points[i].z;
    marker.points[i].z = marker.points[i].x;
    marker.points[i].x = x;
  }


  cost_viz_scaling_const_ = min_cost;
  cost_viz_scaling_factor_ = 0.1 /(max_cost - min_cost);

  int timeout = 5;
  int counter = 0;
  while (rviz_pub_.getNumSubscribers() == 0 && counter < timeout)
  {
    ROS_WARN("Waiting for rviz to connect...");
    ros::Duration(1.0).sleep();
    ++counter;
  }
  rviz_pub_.publish(marker);
}

void Stomp2DTest::visualizeTrajectory(Rollout& rollout, bool noiseless, int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "BASE";
  marker.header.stamp = ros::Time::now();
  marker.ns="trajectory";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points.resize(num_time_steps_);
  //marker.colors.resize(num_time_steps_);
  for (int t=0; t<num_time_steps_; ++t)
  {
    marker.points[t].z = rollout.parameters_noise_[0][t];
    marker.points[t].y = rollout.parameters_noise_[1][t];
    double cost = evaluateMapCost(rollout.parameters_noise_[0][t], rollout.parameters_noise_[1][t]);
    marker.points[t].x = (cost - cost_viz_scaling_const_) * cost_viz_scaling_factor_;
    marker.points[t].x += 0.005;
  }
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  if (noiseless)
  {
    marker.scale.x = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  else
  {
    marker.scale.x = 0.002;
    marker.color.a = 0.7;
    marker.color.r = 0.2;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
  }
  rviz_pub_.publish(marker);
}
*/

} /* namespace stomp */

int main(int argc, char ** argv)
{

    std::string config_file = "../src/planners/stomp/test/stomp_2d_test.yaml";
    std::string obstacles_file = "../src/planners/stomp/test/stomp_2d_test_obstacle.yaml";
    YAML::Node input_config, obstacles_config;
    handle_config::loadConfigFile(config_file, input_config);
    handle_config::loadConfigFile(obstacles_file, obstacles_config);
    // check if we want to do large-scale testing
    bool large_scale = false;
    if (!large_scale)
    {
        std::shared_ptr<stomp::Stomp2DTest> test(new stomp::Stomp2DTest());
        return test->run(input_config, obstacles_config);
    }

/*
  // read params for large scale testing
  int num_dimensions = 2;
  int stomp_repetitions;
  std::string large_scale_output_dir;
  std::string output_dir;
  std::vector<int> stomp_rollouts;
  std::vector<double> stomp_rollouts_dbl;
  std::vector<double> stomp_noises;
  std::vector<double> chomp_learning_rates;
  std::vector<std::string> cost_function_names;
  std::vector<bool> stomp_use_noise_adaptation_or_not;
  std::vector<bool> cost_function_bool_or_not;

  STOMP_VERIFY(node_handle.getParam("large_scale_output_dir", large_scale_output_dir));
  mkdir(large_scale_output_dir.c_str(), 0755);
  STOMP_VERIFY(node_handle.getParam("output_dir", output_dir));
  STOMP_VERIFY(node_handle.getParam("stomp_repetitions", stomp_repetitions));
  STOMP_VERIFY(stomp::readDoubleArray(node_handle, "stomp_rollouts", stomp_rollouts_dbl));
  // convert dbls to int
  for (unsigned int i=0; i<stomp_rollouts_dbl.size(); ++i)
    stomp_rollouts.push_back(lrint(stomp_rollouts_dbl[i]));
  STOMP_VERIFY(stomp::readDoubleArray(node_handle, "stomp_noises", stomp_noises));
  STOMP_VERIFY(stomp::readDoubleArray(node_handle, "chomp_learning_rates", chomp_learning_rates));
  STOMP_VERIFY(stomp::readStringArray(node_handle, "cost_function_names", cost_function_names));

  std::vector<double> tmp_dbl;
  STOMP_VERIFY(stomp::readDoubleArray(node_handle, "stomp_noise_adaptations", tmp_dbl));
  for (unsigned int i=0; i<tmp_dbl.size(); ++i)
  {
    bool val = (tmp_dbl[i]<=0.0)? false: true;
    stomp_use_noise_adaptation_or_not.push_back(val);
  }

  tmp_dbl.clear();
  STOMP_VERIFY(stomp::readDoubleArray(node_handle, "cost_function_bools", tmp_dbl));
  for (unsigned int i=0; i<tmp_dbl.size(); ++i)
  {
    bool val = (tmp_dbl[i]<=0.0)? false: true;
    cost_function_bool_or_not.push_back(val);
  }

  // read all cost functions:
  std::vector<XmlRpc::XmlRpcValue> cost_functions;
  XmlRpc::XmlRpcValue cfs_xml;
  node_handle.getParam("cost_functions", cfs_xml);
  for (unsigned int i=0; i<cost_function_names.size(); ++i)
  {
    XmlRpc::XmlRpcValue cf_xml = cfs_xml[cost_function_names[i]];
    cost_functions.push_back(cf_xml);
  }

  // compute total number of runs
  int num_stomp_runs = stomp_repetitions *
      stomp_rollouts.size() *
      stomp_noises.size() *
      stomp_use_noise_adaptation_or_not.size() *
      cost_function_names.size() *
      cost_function_bool_or_not.size();
  int num_chomp_runs = chomp_learning_rates.size() * cost_function_names.size() * cost_function_bool_or_not.size();
  ROS_INFO("Expected number of STOMP runs: %d", num_stomp_runs);
  ROS_INFO("Expected number of CHOMP runs: %d", num_chomp_runs);

  int run_index = 0;

  // disgusting nested loops
  for (unsigned int cf_index=0; cf_index<cost_function_names.size(); ++cf_index)
  {
    for (unsigned int cb_index=0; cb_index<cost_function_bool_or_not.size(); ++cb_index)
    {
      // convert cost function to bool or not
      for (int i=0; i<cost_functions[cf_index].size(); ++i)
      {
        cost_functions[cf_index][i]["boolean"] = XmlRpc::XmlRpcValue(cost_function_bool_or_not[cb_index] ? true : false);
      }
      // put the cost function on param server
      node_handle.setParam("cost_function", cost_functions[cf_index]);

      // first set stomp mode
      node_handle.setParam("use_chomp", false);

      for (unsigned int sr_index=0; sr_index<stomp_rollouts.size(); ++sr_index)
      {
        //set number of rollouts on param server
        node_handle.setParam("stomp/max_rollouts", stomp_rollouts[sr_index]);
        node_handle.setParam("stomp/min_rollouts", stomp_rollouts[sr_index]);
        node_handle.setParam("stomp/num_rollouts_per_iteration", stomp_rollouts[sr_index]);

        for (unsigned int sn_index=0; sn_index<stomp_noises.size(); ++sn_index)
        {
          //set noise on param server
          XmlRpc::XmlRpcValue noise_array;
          node_handle.getParam("stomp/noise_stddev", noise_array);
          for (int i=0; i<num_dimensions; ++i)
          {
            noise_array[i] = stomp_noises[sn_index];
          }
          node_handle.setParam("stomp/noise_stddev", noise_array);

          for (unsigned int su_index=0; su_index<stomp_use_noise_adaptation_or_not.size(); ++su_index)
          {
            //set stomp/use_noise_adaptation on param server
//S*            node_handle.setParam("stomp/use_noise_adaptation", stomp_use_noise_adaptation_or_not[su_index]);

            for (int srep=0; srep<stomp_repetitions; ++srep)
            {
              // create test name
              std::stringstream test_name;
              if (cost_function_bool_or_not[cb_index])
                test_name << "bool_";
              test_name << cost_function_names[cf_index];
              test_name << "_stomp_";
              test_name << "r" << stomp_rollouts[sr_index];
              test_name << "_n" << stomp_noises[sn_index];
              test_name << (stomp_use_noise_adaptation_or_not[su_index] ? "_na" : "_nona");
              test_name << "_t" << srep;

              ++run_index;

//S*              ROS_INFO_STREAM(test_name.str() << " (" << run_index << "/" << num_stomp_runs+num_chomp_runs << ")");

              // run the test
              boost::shared_ptr<stomp::Stomp2DTest> test(new stomp::Stomp2DTest());
              test->run();

              std::stringstream copy_command;
              copy_command << "mv " << output_dir << " " << large_scale_output_dir << "/" << test_name.str();
              if (system(copy_command.str().c_str()))
              {
//S*                ROS_ERROR("System command failed: %s", copy_command.str().c_str());
                return false;
              }
              //ROS_INFO_STREAM(copy_command.str());
              if (!node_handle.ok())
                return false;
            } // srep
          } // su_index
        } // sn_index
      } // sr_index


    }
  }
*/

}
