#include <wrapper/ompl/RobotFreeJointParameter.hpp>

using namespace motion_planners;

RobotFreeJointParameter::RobotFreeJointParameter(std::string joint_name, double step_size, boost::shared_ptr<RobotModel> &robot_model_ptr)
{
    this->lower_limit=robot_model_ptr->getRobotState().robot_joints[joint_name].getJointInfo().limits->lower;
    this->upper_limit=robot_model_ptr->getRobotState().robot_joints[joint_name].getJointInfo().limits->upper;
    this->step_size=step_size;
    //this->pivot=(this->upper_limit+this->lower_limit)/2;
    this->pivot=robot_model_ptr->getRobotState().robot_joints[joint_name].getJointValue();
    this->joint_name=joint_name;
    joint_position_has_been_set=false;
}


RobotFreeJointParameter::RobotFreeJointParameter(std::string joint_name, double step_size,double pivot ,boost::shared_ptr<RobotModel> &robot_model_ptr)
{

    this->lower_limit=robot_model_ptr->getRobotState().robot_joints[joint_name].getJointInfo().limits->lower;
    this->upper_limit=robot_model_ptr->getRobotState().robot_joints[joint_name].getJointInfo().limits->upper;
    this->step_size=step_size;
    this->pivot=pivot;
    this->joint_name=joint_name;
    joint_position_has_been_set=false;

}


RobotFreeJointParameter::RobotFreeJointParameter(std::string joint_name,double position )
{
    this->joint_position=position;
//    this->upper_limit=position;
//    this->step_size=std::numeric_limits<double>::max();
//    this->pivot=std::numeric_limits<double>::max();
    this->joint_name=joint_name;
    joint_position_has_been_set=true;
}

void RobotFreeJointParameter::setPivot(double &pivot)
{
    this->pivot=pivot;
}

double RobotFreeJointParameter::getPivot()
{
    return this->pivot;
}

void RobotFreeJointParameter::setStepSize(double &step_size)
{
    this->step_size=step_size;
}

double RobotFreeJointParameter::getStepSize()
{
    return this->step_size;
}

void RobotFreeJointParameter::setUpperLimit(double &upper_limit)
{
    this->upper_limit=upper_limit;
}

double RobotFreeJointParameter::getUpperLimit()
{
    return this->upper_limit;
}

void  RobotFreeJointParameter::setLowerLimit(double &lower_limit)
{
    this->lower_limit=lower_limit;
}

double RobotFreeJointParameter::getLowerLimit()
{
    return this->lower_limit;
}

std::string & RobotFreeJointParameter::getJointName()
{
    return this->joint_name;
}

void RobotFreeJointParameter::setJointName(std::string & joint_name)
{
    this->joint_name=joint_name;
}


bool RobotFreeJointParameter::jointPositionHasBeenSet()
{
    return this->joint_position_has_been_set;
}


double RobotFreeJointParameter::getJointPosition()
{
    return this->joint_position;
}
