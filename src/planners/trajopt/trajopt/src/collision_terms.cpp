#include "trajopt/collision_terms.hpp"
#include "trajopt/collision_checker.hpp"
//#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "sco/expr_vec_ops.hpp"
#include "sco/expr_ops.hpp"
#include "sco/sco_common.hpp"
#include <boost/foreach.hpp>
#include "utils/eigen_conversions.hpp"
#include "sco/modeling_utils.hpp"
#include "utils/stl_to_string.hpp"
#include "utils/logging.hpp"
#include <boost/functional/hash.hpp>

using namespace sco;
using namespace util;
using namespace std;

namespace trajopt {


void CollisionsToDistances(const vector<Collision>& collisions,  DblVec& dists) {
  // Note: this checking (that the links are in the list we care about) is probably unnecessary
  // since we're using LinksVsAll
  dists.clear();
  dists.reserve(collisions.size());
//  BOOST_FOREACH(const Collision& col, collisions) {
//    Link2Name::const_iterator itA =  link2Name.find(col.linkA);
//    Link2Name::const_iterator itB =  link2Name.find(col.linkB);
//    if (itA !=  link2Name.end() || itB !=  link2Name.end()) {
//      dists.push_back(col.distance);
//    }
//  }

  BOOST_FOREACH(const Collision& col, collisions) {
        dists.push_back(col.distance);
    }

}

void CollisionsToDistanceExpressions(const vector<Collision>& collisions, ConfigurationPtr& rad,
    const VarVector& vars, const DblVec& dofvals, vector<AffExpr>& exprs, bool isTimestep1) {

  exprs.clear();
  exprs.reserve(collisions.size());
  rad->SetDOFValues(dofvals); // since we'll be calculating jacobians
  BOOST_FOREACH(const Collision& col, collisions) {
    AffExpr dist(col.distance);
//    Link2Name::const_iterator itA = link2Name.find(col.linkA);
//    if (itA != link2Name.end()) {
//      VectorXd dist_grad = col.normalB2A.transpose()*rad->PositionJacobian(itA->second, col.ptA);
//      exprInc(dist, varDot(dist_grad, vars));
//      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
//    }
//    Link2Name::const_iterator itB = link2Name.find(col.linkB);
//    if (itB != link2Name.end()) {
//      VectorXd dist_grad = -col.normalB2A.transpose()*rad->PositionJacobian(itB->second, (isTimestep1 && (col.cctype == CCType_Between)) ? col.ptB1 : col.ptB);
//      exprInc(dist, varDot(dist_grad, vars));
//      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
//    }
//    if (itA != link2Name.end() || itB != link2Name.end()) {
//      exprs.push_back(dist);
//    }

//    Link2Name::const_iterator itA = link2Name.find(col.linkA);
    VectorXd dist_grad = -col.normalB2A.transpose()*rad->PositionJacobian(col.linkA, col.ptA);
    exprInc(dist, varDot(dist_grad, vars));
    exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
    exprs.push_back(dist);
  }
  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}

void CollisionsToDistanceExpressions(const vector<Collision>& collisions, ConfigurationPtr& rad,
    const VarVector& vars0, const VarVector& vars1, const DblVec& vals0, const DblVec& vals1,
    vector<AffExpr>& exprs) {
  vector<AffExpr> exprs0, exprs1;
  CollisionsToDistanceExpressions(collisions, rad, vars0, vals0, exprs0, false);
  CollisionsToDistanceExpressions(collisions, rad, vars1, vals1, exprs1,true);

  exprs.resize(exprs0.size());
  for (int i=0; i < exprs0.size(); ++i) {
    exprScale(exprs0[i], (1-collisions[i].time));
    exprScale(exprs1[i], collisions[i].time);
    exprs[i] = AffExpr(0);
    exprInc(exprs[i], exprs0[i]);
    exprInc(exprs[i], exprs1[i]);
    cleanupAff(exprs[i]);
  }
}

inline size_t hash(const DblVec& x) {
  return boost::hash_range(x.begin(), x.end());
}

void CollisionEvaluator::GetCollisionsCached(const DblVec& x, vector<Collision>& collisions) {
  double key = hash(getDblVec(x, GetVars()));
  vector<Collision>* it = m_cache.get(key);
  if (it != NULL) {
    LOG_DEBUG("using cached collision check\n");
    collisions = *it;
  }
  else {
    LOG_DEBUG("not using cached collision check\n");
    CalcCollisions(x, collisions);
    m_cache.put(key, collisions);
  }
}

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(ConfigurationPtr rad, CollisionCheckerPtr coll, const VarVector& vars) :
  m_rad(rad),
  m_cc(coll),
  m_vars(vars),
  m_links(),
  m_filterMask(-1) {
//  vector<LinkPtr> links;
//  vector<std::string> link_names;
//  rad->GetAffectedLinks(m_links, true, link_names);
//  for (int i=0; i < m_links.size(); ++i) {
//    m_link2Name[m_links[i].get()] = link_names[i];
//  }
}


void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Collision>& collisions) {
  DblVec dofvals = getDblVec(x, m_vars);
  m_rad->SetDOFValues(dofvals);
  m_cc->GetDiscreteCollisionInfo(collisions);
}

void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  CollisionsToDistances(collisions, dists);
}


void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  DblVec dofvals = getDblVec(x, m_vars);
  CollisionsToDistanceExpressions(collisions, m_rad, m_vars, dofvals, exprs, false);
}

//////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(ConfigurationPtr rad, CollisionCheckerPtr coll, const VarVector& vars0, const VarVector& vars1):
  m_rad(rad),
  m_cc(coll),
  m_vars0(vars0),
  m_vars1(vars1),
  m_links()
{
//  vector<LinkPtr> links;
//  vector<std::string> link_names;
//  rad->GetAffectedLinks(m_links, true, link_names);
//  for (int i=0; i < m_links.size(); ++i) {
//    m_link2Name[m_links[i].get()] = link_names[i];
//  }
}

void CastCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Collision>& collisions) {
  DblVec dofvals0 = getDblVec(x, m_vars0);
  DblVec dofvals1 = getDblVec(x, m_vars1);
  m_rad->SetDOFValues(dofvals0);
  m_cc->GetContinuousCollisionInfo(dofvals0, dofvals1, collisions);
}
void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  DblVec dofvals0 = getDblVec(x, m_vars0);
  DblVec dofvals1 = getDblVec(x, m_vars1);
  CollisionsToDistanceExpressions(collisions, m_rad, m_vars0, m_vars1, dofvals0, dofvals1, exprs);
}
void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  CollisionsToDistances(collisions, dists);
}

CollisionCost::CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, CollisionCheckerPtr coll, const VarVector& vars) :
    Cost("collision"),
    m_calc(new SingleTimestepCollisionEvaluator(rad, coll, vars)), m_dist_pen(dist_pen), m_coeff(coeff)
{}

CollisionCost::CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, CollisionCheckerPtr coll, const VarVector& vars0, const VarVector& vars1) :
    Cost("cast_collision"),
    m_calc(new CastCollisionEvaluator(rad, coll, vars0, vars1)), m_dist_pen(dist_pen), m_coeff(coeff)
{}
ConvexObjectivePtr CollisionCost::convex(const vector<double>& x, Model* model) {
    LOG_DEBUG("CollisionCost::convex . . . .. . . . . \n");

  ConvexObjectivePtr out(new ConvexObjective(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);
  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
    out->addHinge(viol, m_coeff);
  }
  return out;
}
double CollisionCost::value(const vector<double>& x) {

  LOG_DEBUG("CollisionCost::value . . . .. . . . . .. . . . . . .\n");
  DblVec dists;
  m_calc->CalcDists(x, dists);
  double out = 0;
  for (int i=0; i < dists.size(); ++i) {
    out += pospart(m_dist_pen - dists[i]) * m_coeff;
  }

//  LOG_INFO("CollisionCost::value --------------------- %.3e ", out);

  return out;
}

//// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad, CollisionCheckerPtr coll, const VarVector& vars) :
    m_calc(new SingleTimestepCollisionEvaluator(rad, coll, vars)), m_dist_pen(dist_pen), m_coeff(coeff)
{
  name_="collision";
}

CollisionConstraint::CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad, CollisionCheckerPtr coll, const VarVector& vars0, const VarVector& vars1) :
    m_calc(new CastCollisionEvaluator(rad, coll, vars0, vars1)), m_dist_pen(dist_pen), m_coeff(coeff)
{
  name_="collision";
}
ConvexConstraintsPtr CollisionConstraint::convex(const vector<double>& x, Model* model) {
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);
  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
    out->addIneqCnt(exprMult(viol,m_coeff));
  }
  return out;
}
DblVec CollisionConstraint::value(const vector<double>& x) {
  DblVec dists;
  m_calc->CalcDists(x, dists);
  DblVec out(dists.size());
  for (int i=0; i < dists.size(); ++i) {
    out[i] = pospart(m_dist_pen - dists[i]) * m_coeff;
  }
  return out;
}



}
