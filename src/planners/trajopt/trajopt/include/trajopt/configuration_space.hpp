#pragma once
#include "typedefs.hpp"
#include "utils/macros.h"
#include "trajopt/transform.h"
#include "trajopt/trajoptinterface.h"

namespace trajopt {
///**
//Stores an OpenRAVE robot and the active degrees of freedom
//*/
//class TRAJOPT_API RobotAndDOF : public Configuration {
//public:
//  RobotAndDOF(OR::KinBodyPtr _robot, const IntVec& _joint_inds, int _affinedofs=0, const OR::Vector _rotationaxis=OR::Vector(0,0,1)) :
//    robot(_robot), joint_inds(_joint_inds), affinedofs(_affinedofs), rotationaxis(_rotationaxis) {}

//  void SetDOFValues(const DblVec& dofs);
//  void GetDOFLimits(DblVec& lower, DblVec& upper) const;
//  DblVec GetDOFValues();
//  int GetDOF() const;
//  virtual OpenRAVE::EnvironmentBasePtr GetEnv() {return robot->GetEnv();};
//  IntVec GetJointIndices() const {return joint_inds;}
//  DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const;
//  DblMatrix RotationJacobian(int link_ind) const;
//  OR::RobotBasePtr GetRobot() const {return boost::dynamic_pointer_cast<RobotBase>(robot);}
//  virtual vector<OpenRAVE::KinBodyPtr> GetBodies();
//  bool DoesAffect(const KinBody::Link& link);
//  std::vector<KinBody::LinkPtr> GetAffectedLinks();
//  void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds);
//  DblVec RandomDOFValues();

//  struct RobotSaver : public Saver {
//    OpenRAVE::KinBody::KinBodyStateSaver saver;
//    RobotSaver(OpenRAVE::KinBodyPtr robot) : saver(robot) {}
//  };
//  SaverPtr Save() {
//    return SaverPtr(new RobotSaver(robot));
//  }
//  void SetRobotActiveDOFs();
  
//private:
//  OpenRAVE::KinBodyPtr robot;
//  IntVec joint_inds;
//  int affinedofs;
//  OR::Vector rotationaxis;
//};
//typedef boost::shared_ptr<RobotAndDOF> RobotAndDOFPtr;

}
