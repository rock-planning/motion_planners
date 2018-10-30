#ifndef ABSTRACTPLANNER_HPP_
#define ABSTRACTPLANNER_HPP_

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include "RobotModel.hpp"
#include <base/JointsTrajectory.hpp>
#include <kinematics_library/KinematicsConfig.hpp>

namespace motion_planners
{

inline void loadConfigFile(std::string filename, YAML::Node &config)
{
    try
    {
        config = YAML::LoadFile(filename);
    }
    catch (YAML::ParserException& e)
    {
        std::cout << e.what() << "\n";
    }


}

template<typename T>
T getValue (const YAML::Node &yaml_data, std::string name)
{
        T value;

        if (const YAML::Node data = yaml_data[name])
        {
            value = data.as<T>();
        }
        else
            std::cout << "Key "<< name <<" doesn't exist\n";

        return value; 

}

template<typename T>
T getValue (const YAML::Node &yaml_data, std::string name, const T& df)
{
        T value;

        if (const YAML::Node data = yaml_data[name])
        {
            value = data.as<T>();
        }
        else
            value = df;

        return value;

}

// enum PlannerLibrary
// {
//     STOMP, OMPL
// };

struct Limits
{
    double min;
    double max;
};

struct CartesianContraints
{
    Limits x,y,z, rx, ry, rz;
};

struct PlannerStatus
{
    enum StatusCode
    {
        /// Planner found a path
        PATH_FOUND,
        /// No path found
        NO_PATH_FOUND,
        /// Start state is in collision
        START_STATE_IN_COLLISION,
        /// Goal state is in collision
        GOAL_STATE_IN_COLLISION,
        /// Start joint angle is not available
        START_JOINTANGLES_NOT_AVAILABLE,
        /// Goal joint angle is not available
        GOAL_JOINTANGLES_NOT_AVAILABLE,
        /// The constraint's upper and lower bounds are not within bounds
        /// OMPL expect lower bounds lesser than the upper bounds
        CONSTRAINED_POSE_NOT_WITHIN_BOUNDS,        
        /// Planning reuest is successfully created.
        PLANNING_REQUEST_SUCCESS,
        // The planner timeout
        TIMEOUT,
        // Invalid start state or no start state specified
        INVALID_START_STATE,
        // Invalid goal state
        INVALID_GOAL_STATE,
        // The goal is of a type that a planner does not recognize
        UNRECOGNIZED_GOAL_TYPE,
        // The planner found an approximate solution
        APPROXIMATE_SOLUTION,
        // The planner found an exact solution
        EXACT_SOLUTION,
        // Robot model initialisation failed
        ROBOTMODEL_INITIALISATION_FAILED,
        // Planner initialisation failed
        PLANNER_INITIALISATION_FAILED,
        /// The planner crashed
        CRASH,
        /// invalid state
        INVALID
    }statuscode;

    kinematics_library::KinematicsStatus kinematic_status;

};

class AbstractPlanner
{

public:

    AbstractPlanner(){};
    virtual ~AbstractPlanner(){};

    /**
     * Initialize the task for a given number of threads.
     * @param num_threads Number of threads for multi-threading
     * @return
     */
    virtual bool initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string planner_specfic) = 0;

    /**
     * Executes the task for the given policy parameters, and returns the costs per timestep
     * Must be thread-safe!
     * @param parameters [num_dimensions] num_parameters - policy parameters to execute
     * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
     * @param weighted_feature_values num_time_steps x num_features matrix of weighted feature values per time step
     * @return
     */
//     virtual bool execute(std::vector<Eigen::VectorXd>& parameters,
//                          std::vector<Eigen::VectorXd>& projected_parameters,
//                          Eigen::VectorXd& costs,
//                          Eigen::MatrixXd& weighted_feature_values,
//                          const int iteration_number,
//                          const int rollout_number,
//                          int thread_id,
//                          bool compute_gradients,
//                          std::vector<Eigen::VectorXd>& gradients,
//                          bool& validity) = 0;

    /**
     * Filters the given parameters - for eg, clipping of joint limits
     * Must be thread-safe!
     * @param parameters
     * @return false if no filtering was done
     */
//     virtual bool filter(std::vector<Eigen::VectorXd>& parameters, int rollout_id, int thread_id) {return false;};

    /**
     * Gets the weight of the control cost
     * @return
     */
//     virtual double getControlCostWeight() = 0;

    /**
     * Callback executed after each iteration
     */
//     virtual void onEveryIteration(){};

    virtual bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status) = 0;
    virtual void updateInitialTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal, PlannerStatus &planner_status) = 0;

};

typedef std::shared_ptr<AbstractPlanner> AbstractPlannerPtr;

}
#endif /* STOMPTASK_H_ */
