#ifndef TRAJOPTINTERFACE_H
#define TRAJOPTINTERFACE_H

#pragma once
#include "typedefs.hpp"
#include "utils/macros.h"
#include "trajopt/transform.h"

namespace trajopt {

typedef std::string Link;

enum CastCollisionType {
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};

struct Collision {
  std::string linkA;
  std::string linkB;
  Vector3d ptA, ptB, normalB2A; /* normal points from 2 to 1 */
  Vector3d ptB1;
  double distance; /* pt1 = pt2 + normal*dist */
  float weight, time;
  CastCollisionType cctype;
  Collision(const std::string linkA, std::string linkB, const Vector3d& ptA, const Vector3d& ptB, const Vector3d& normalB2A, double distance, float weight=1, float time=0) :
    linkA(linkA), linkB(linkB), ptA(ptA), ptB(ptB), normalB2A(normalB2A), distance(distance), weight(weight), time(0), cctype(CCType_None) {}
  Collision(): time(0), cctype(CCType_None) {}
};
TRAJOPT_API std::ostream& operator<<(std::ostream&, const Collision&);

enum CollisionFilterGroups {
  RobotFilter = 1,
  KinBodyFilter = 2,
};

class TRAJOPT_API Configuration {
public:

    virtual void setDOFValues(const DblVec& dofs) = 0;
    virtual void getDOFLimits(DblVec& lower, DblVec& upper) const = 0;
    virtual DblVec getDOFValues() = 0;
    virtual int getDOF() const = 0;
    virtual DblMatrix getPositionJacobian(const std::string link_name, const Vector3d& pt) /*const*/ = 0;
    virtual DblMatrix getRotationJacobian(const std::string link_name) const = 0;
    virtual DblVec setRandomDOFValues() = 0;
    virtual geometry::Transform getLinkTransformByName(const std::string link_name) = 0;
    virtual bool checkIfLinkExists(const std::string link_name) = 0;
//    virtual boost::shared_ptr<CollisionChecker> getCollisionChecker() = 0;

    virtual ~Configuration() {}
};
typedef boost::shared_ptr<Configuration> ConfigurationPtr;

/**
Each CollisionChecker object has a copy of the world, so for performance, don't make too many copies
*/
class TRAJOPT_API CollisionChecker {
public:

    /** contacts of distance < (arg) will be returned */
    virtual void setDistanceTolerance(float distance)  = 0;
    virtual double getContactDistance() = 0;

    /** Find contacts between swept-out shapes of robot links and everything in the environment, as robot goes from startjoints to endjoints */
    virtual void getContinuousCollisionInfo(const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions) = 0;

    virtual void getDiscreteCollisionInfo(vector<Collision> &collisions) = 0;

    virtual ~CollisionChecker() {}
    /** Get or create collision checker for this environment */
//    static boost::shared_ptr<CollisionChecker> GetOrCreate();
protected:
    CollisionChecker(){}
};
typedef boost::shared_ptr<CollisionChecker> CollisionCheckerPtr;

CollisionCheckerPtr TRAJOPT_API CreateCollisionChecker();

}

#endif // TRAJOPTINTERFACE_H
