#pragma once
#include <string>
#include "modeling.hpp"
#include <boost/function.hpp>
#include "sco/SQPConfig.h"

/*
 * Algorithms for non-convex, constrained optimization
 */

namespace sco {

using std::string;
using std::vector;


enum OptStatus {
  OPT_CONVERGED,
  OPT_SCO_ITERATION_LIMIT, // hit iteration limit before convergence
  OPT_PENALTY_ITERATION_LIMIT,
  OPT_FAILED,
  INVALID
};
static const char* OptStatus_strings[]  = {
  "CONVERGED",
  "SCO_ITERATION_LIMIT",
  "PENALTY_ITERATION_LIMIT",
  "FAILED",
  "INVALID"
};
inline string statusToString(OptStatus status) {
  return OptStatus_strings[status];
}


struct OptResults {
  DblVec x; // solution estimate
  OptStatus status;
  double total_cost;
  vector<double> cost_vals;
  DblVec cnt_viols;
  int n_func_evals, n_qp_solves;
  void clear() {
    x.clear();
    status = INVALID;
    cost_vals.clear();
    cnt_viols.clear();
    n_func_evals = 0;
    n_qp_solves = 0;
  }
  OptResults() {clear();}
};
std::ostream& operator<<(std::ostream& o, const OptResults& r);

class Optimizer {
  /*
   * Solves an optimization problem
   */
public:
  virtual OptStatus optimize() = 0;
  virtual ~Optimizer() {}
  virtual void setProblem(OptProbPtr prob) {prob_ = prob;}
  void initialize(const vector<double>& x);
  vector<double>& x() {return results_.x;}
  OptResults& results() {return results_;}

  typedef boost::function<void(OptProb*, DblVec&)> Callback;
  void addCallback(const Callback& f); // called before each iteration
protected:
  vector<Callback> callbacks_;
  void callCallbacks(DblVec& x);
  OptProbPtr prob_;
  OptResults results_;
};

class BasicTrustRegionSQP : public Optimizer {
  /*
   * Alternates between convexifying objectives and constraints and then solving convex subproblem
   * Uses a merit function to decide whether or not to accept the step
   * merit function = objective + merit_err_coeff * | constraint_error |
   * Note: sometimes the convexified objectives and constraints lead to an infeasible subproblem
   * In that case, you should turn them into penalties and solve that problem
   * (todo: implement penalty-based sqp that gracefully handles infeasible constraints)
   */
public:
  SQPConfig sqp_config;
  BasicTrustRegionSQP();
  BasicTrustRegionSQP(OptProbPtr prob);
  BasicTrustRegionSQP(OptProbPtr prob, SQPConfig sqp_config);
  void setProblem(OptProbPtr prob);
  OptStatus optimize();
protected:
  void adjustTrustRegion(double ratio);
  void setTrustBoxConstraints(const vector<double>& x);
  ModelPtr model_;
};


}
