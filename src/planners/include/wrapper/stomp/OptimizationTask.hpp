
#ifndef OPTIMIZATIONTASK_HPP_
#define OPTIMIZATIONTASK_HPP_

#include <stomp/Stomp.hpp>
#include <stomp/StompTask.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <base/samples/Joints.hpp>
#include <RobotModel.hpp>

namespace motion_planners
{
  

class OptimizationTask: public stomp::StompTask, public boost::enable_shared_from_this<motion_planners::OptimizationTask>
{
    public:
        OptimizationTask(stomp::StompConfig config, std::shared_ptr<RobotModel>& robot_model);

        virtual ~OptimizationTask();


        // functions inherited from Task:

        /**
         * Initialize the task for a given number of threads.
         * @param num_threads Number of threads for multi-threading
         * @return
         */
        
        bool stompInitialize(int num_threads, int num_rollouts);
        
        //void updateInitialTrajectory(const base::VectorXd &start, const base::VectorXd &goal);
        void updateTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal);
                
        void createPolicy();
        
        void updatePolicy();

        /**
         * Executes the task for the given policy parameters, and returns the costs per timestep
         * @param parameters [num_dimensions] num_parameters - policy parameters to execute
         * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
         * @param weighted_feature_values num_time_steps x num_features matrix of weighted feature values per time step
         * @return
         */
        virtual bool execute(std::vector<base::VectorXd>& parameters,
                             std::vector<base::VectorXd>& projected_parameters,
                             base::VectorXd& costs,
                             base::MatrixXd& weighted_feature_values,
                             const int iteration_number,
                             const int rollout_number,
                             int thread_id,
                             bool compute_gradients,
                             std::vector<base::VectorXd>& gradients,
                             bool& validity);

        virtual bool filter(std::vector<base::VectorXd>& parameters, int rollout_id, int thread_id);

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

        boost::shared_ptr<stomp::CovariantMovementPrimitive> policy_;        

        std::vector<base::VectorXd> initial_trajectory_;
    private:

        stomp::StompConfig stomp_config_;

        double movement_dt_;

        base::MatrixXd vel_diff_matrix_;
        base::MatrixXd acc_diff_matrix_;

        std::vector<base::MatrixXd> derivative_costs_;

        base::MatrixXd proj_pos_, pos_;
        base::MatrixXd vel_, acc_;


        base::VectorXd collision_costs_;


        void computeCollisionCost();
        
        std::shared_ptr<RobotModel> robot_model_;
        std::string planning_group_name_;
        std::vector< std::string> planning_group_joints_names_;
        std::vector< double > lower_limits_;
        std::vector< double > upper_limits_;
    
    
};
}// end planner

#endif
