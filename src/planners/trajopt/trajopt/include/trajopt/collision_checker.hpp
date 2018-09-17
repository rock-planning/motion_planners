#pragma once
#include "typedefs.hpp"
#include <set>
#include <utility>
#include <iostream>
//#include <openrave/openrave.h>
#include "trajoptinterface.h"
#include "utils/macros.h"

namespace trajopt {

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

//TRAJOPT_API void PlotCollisions(const std::vector<Collision>& collisions, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles, double safe_dist);

}

