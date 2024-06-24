#ifndef ABSTRACTPLANNER_HPP_
#define ABSTRACTPLANNER_HPP_

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include "AbstractPlannerConfig.hpp"
#include <robot_model/RobotModel.hpp>
#include <base/JointsTrajectory.hpp>
// #include <kinematics_library/KinematicsConfig.hpp>

namespace motion_planners
{
/**
* @brief  Load configuration file.
* @param  filename Configuration file.
* @param  config Output yaml node.
*/
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

template<typename T, typename O>
T getValue (const YAML::Node &yaml_data, std::string name)
{
        T value;

        if (const YAML::Node data = yaml_data[name])
        {
            try {
                value = data.as<T>();
            } catch (const std::exception &e) {
                value = static_cast<T>(data.as<O>());
            }
        }
        else
            std::cout << "Key "<< name <<" doesn't exist\n";

        return value;

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

class AbstractPlanner
{

    public:
        /**
        * @brief Abstract class constructor
        */
        AbstractPlanner();
        /**
        * @brief Destructor
        */
        virtual ~AbstractPlanner(){};
        /**
        * @brief Initialize the motion planner
        * @param robot_model Robot model.
        * @param planner_specfic Configuration file for planner.
        * @return Returns true, when the initialization is successful or else returns false
        */
        virtual bool initializePlanner(std::shared_ptr<robot_model::RobotModel>& robot_model, std::string planner_specfic) = 0;
        /**
        * @brief Reinitialize the motion planner. This function is used in STOMP planner.
        * @return Returns true, when the initialization is successful or else returns false
        */
        virtual bool reInitializePlanner() = 0;
        /**
        * @brief Reinitialize the time step in the motion planner. This function is used in STOMP planner.
        */
        virtual bool reInitializeTimeSteps(const int &num_time_steps) = 0;
        /**
        * @brief  Solve the planning problem.
        * @param  solution Planner solution.
        * @param  planner_status Planner status.        
        * @return Returns true, when the planning is successful else returns false.
        */
        virtual bool solve(base::JointsTrajectory &solution, PlannerStatus &planner_status) = 0;
        /**
        * @brief  Set the start and goal to the planner.
        * @param  start Start configuration in joint space. 
        * @param  goal Goal configuration in joint space.        
        */
        virtual void setStartGoalTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal) = 0;
        /**
        * @brief  Set the contraints for the planner. This function is used in OMPL planner.
        * @param  constraints Constraints.        
        */
        virtual void setConstraints(const ConstraintPlanning constraints) = 0;
        /**
        * @brief  Update the initial trajectory. This function is used only for the STOMP planner.
        * @param  trajectory Initial trajectory.
        * @return Returns true, when the update is successful or else returns false.
        */
        virtual bool updateInitialTrajectory(const base::JointsTrajectory &trajectory) = 0;
        /**
        * @brief  Get the initial trajectory. This function is used only for the STOMP planner.        
        * @return Initial trajectory.
        */
        virtual base::JointsTrajectory getInitialTrajectory() = 0;
        /**
        * @brief  Get the number of iteration used for the planning problem. This function is used only for the STOMP planner.        
        * @return Number of iteration.
        */
        virtual size_t getNumOfIterationsUsed() = 0;        

    protected:
        std::shared_ptr<robot_model::RobotModel> robot_model_;
        std::string planning_group_name_;
        std::vector< std::string> planning_group_joints_name_;
        std::vector< std::pair<std::string,urdf::Joint> > planning_group_joints_;
        std::string root_name_, base_name_, tip_name_;
        
        bool assignPlanningJointInformation(std::shared_ptr<robot_model::RobotModel> robot_model);
};

typedef std::shared_ptr<AbstractPlanner> AbstractPlannerPtr;

}
#endif /* STOMPTASK_H_ */
