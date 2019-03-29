#ifndef ROBOTFREEJOINTPARAMETER_HPP
#define ROBOTFREEJOINTPARAMETER_HPP
#include <string>
#include <boost/shared_ptr.hpp>

#include <robot_model/RobotModel.hpp>

namespace motion_planners
{
class RobotModel;

class RobotFreeJointParameter 
{
    double pivot;
    double step_size;
    double lower_limit;
    double upper_limit;
    std::string joint_name;
    double joint_position;
    bool joint_position_has_been_set;


public:
    RobotFreeJointParameter(std::string joint_name, double step_size, boost::shared_ptr<RobotModel> &robot_model_ptr);
    RobotFreeJointParameter(std::string joint_name, double step_size,double pivot ,boost::shared_ptr<RobotModel> &robot_model_ptr);
    RobotFreeJointParameter(std::string joint_name, double position );
    RobotFreeJointParameter();

    void setPivot(double &pivot);

    double getPivot();

    void setStepSize(double &step_size);

    double getStepSize();

    void setUpperLimit(double &upper_limit);

    double getUpperLimit();

    void  setLowerLimit(double &lower_limit);

    double getLowerLimit();

    std::string & getJointName();

    void setJointName(std::string & joint_name);

    bool jointPositionHasBeenSet();

    double getJointPosition();
};

}// end namespace manipulator_planner_library

#endif //ROBOTFREEJOINTPARAMETER_HPP
