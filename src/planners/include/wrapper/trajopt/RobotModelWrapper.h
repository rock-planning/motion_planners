#ifndef ROBOTMODELWRAPPER_H
#define ROBOTMODELWRAPPER_H

#include "trajopt/trajoptinterface.h"
#include <RobotModel.hpp>
#include <vector>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;

typedef std::vector<double> DblVec;

using namespace trajopt;
using namespace motion_planners;

class RobotModelWrapper : public Configuration
{
    std::shared_ptr<RobotModel> m_robot_model;
    std::string m_planning_group_name_;
    std::vector< std::string> m_planning_group_joints_names_;
    std::vector< double > m_lower_limits_;
    std::vector< double > m_upper_limits_;

public:

    RobotModelWrapper(){}
    RobotModelWrapper(std::shared_ptr<RobotModel> &robot_model);

    virtual ~RobotModelWrapper() {}


//    void setRobotModel(std::shared_ptr<RobotModel> robot_model){ m_robot_model = robot_model; }
    void setRobotModel(std::shared_ptr<RobotModel> robot_model);
    virtual void setDOFValues(const DblVec& dofs);
    virtual void getDOFLimits(DblVec& lower, DblVec& upper) const;
    virtual DblVec getDOFValues();
    virtual int getDOF() const;
    virtual DblMatrix getPositionJacobian(std::string link_name, const Vector3d& pt) /*const*/;
    virtual DblMatrix getRotationJacobian(std::string link_name) const;
    virtual DblVec setRandomDOFValues();
    virtual geometry::Transform getLinkTransformByName(std::string link_name);
    virtual bool checkIfLinkExists(const std::string link_name);

    void dblVecToEigenVectorXd(const DblVec &in, Eigen::VectorXd &out);
    void getDOFValues(vector<std::string> &joint_names, DblVec &joint_values);
    void getDOFValues(std::map<std::string, double> &joint_states_map);
};
typedef boost::shared_ptr<RobotModelWrapper> RobotModelWrapperPtr;

#endif // ROBOTMODELWRAPPER_H
