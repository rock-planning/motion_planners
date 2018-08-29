#ifndef ROBOTMODELWRAPPER_H
#define ROBOTMODELWRAPPER_H

#include "trajopt/configuration_space.hpp"
#include "wrapper/trajopt/FCLCollisionChecker.h"
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
    virtual void SetDOFValues(const DblVec& dofs);
    virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const;
    virtual DblVec GetDOFValues();
    virtual int GetDOF() const;
    virtual DblMatrix PositionJacobian(std::string link_name, const Vector3d& pt) const;
    virtual DblMatrix RotationJacobian(std::string link_name) const;
//    virtual bool DoesAffect(const KinBody::Link& link)  {}
//    virtual vector<T> GetBodies() {}
//    virtual std::vector<T> GetAffectedLinks()  {}
    virtual void GetAffectedLinks(std::vector<LinkPtr>& links, bool only_with_geom, vector<std::string>& link_inds);
    virtual DblVec RandomDOFValues();
    virtual geometry::Transform GetLinkTransformByName(std::string link_name);

//    struct Saver {
//      virtual ~Saver(){}
//    };
//    typedef boost::shared_ptr<Saver> SaverPtr;
//    struct GenericSaver : public Saver {
//      DblVec dofvals;
//      Configuration* parent;
//      GenericSaver(Configuration* _parent) : dofvals(_parent->GetDOFValues()), parent(_parent) {}
//      ~GenericSaver() {
//        parent->SetDOFValues(dofvals);
//      }
//    }; // inefficient

//    virtual SaverPtr Save() {
//      return SaverPtr(new GenericSaver(this));
//    }

    void DblVecToEigenVectorXd(const DblVec &in, Eigen::VectorXd &out);
    void GetDOFValues(vector<std::string> &joint_names, DblVec &joint_values);
    void GetDOFValues(std::map<std::string, double> &joint_states_map);
};
typedef boost::shared_ptr<RobotModelWrapper> RobotModelWrapperPtr;

#endif // ROBOTMODELWRAPPER_H
