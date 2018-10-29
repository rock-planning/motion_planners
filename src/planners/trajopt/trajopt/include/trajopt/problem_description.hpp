#pragma once
#include "trajopt/common.hpp"
#include "json_marshal.hpp"
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
//#include "traj_plotter.hpp"
#include "sco/modeling.hpp"
#include "yaml-cpp/yaml.h"
#include "sco/expr_ops.hpp"
#include "sco/optimizers.hpp"

using namespace sco;
namespace sco{struct OptResults; /*struct ProblemConstructionInfo;*/}

namespace Json {
Json::Value readJsonFile(const std::string& fname);
}


//namespace YAML {
//void readYamlFile(const std::string& fname, YAML::Node config);
//}

namespace trajopt {

void readYamlFile(const char* filename, YAML::Node config);

using namespace json_marshal;
using namespace Json;
typedef Json::Value TrajOptRequest;
typedef Json::Value TrajOptResponse;
using std::string;

struct TermInfo;
typedef boost::shared_ptr<TermInfo> TermInfoPtr;
class TrajOptProb;
typedef boost::shared_ptr<TrajOptProb> TrajOptProbPtr;
struct ProblemConstructionInfo;
struct TrajOptResult;
typedef boost::shared_ptr<TrajOptResult> TrajOptResultPtr;

TrajOptProbPtr TRAJOPT_API ConstructProblem(const ProblemConstructionInfo&, SQPConfig config);
TrajOptProbPtr TRAJOPT_API ConstructProblem(const Json::Value&, ConfigurationPtr rad, CollisionCheckerPtr coll);
TrajOptProbPtr TRAJOPT_API ConstructProblem(const YAML::Node &, ConfigurationPtr rad, CollisionCheckerPtr coll);
TrajOptResultPtr TRAJOPT_API OptimizeProblem(TrajOptProbPtr, SQPConfig config = SQPConfig());
SQPConfig getSQPConfig(const Value &root);
SQPConfig getSQPConfig(const YAML::Node &root);

TRAJOPT_API TrajArray getStraightLineTrajData(int n_steps, int n_dof, DblVec startpoint, DblVec endpoint);
TRAJOPT_API TrajArray getStationaryTrajData(DblVec startpoint);

enum TermType {
  TT_COST,
  TT_CNT
};

#define DEFINE_CREATE(classname) \
  static TermInfoPtr create() {\
    TermInfoPtr out(new classname());\
    return out;\
  }
  

/**
 * Holds all the data for a trajectory optimization problem
 * so you can modify it programmatically, e.g. add your own costs
 */

class TRAJOPT_API TrajOptProb : public OptProb {
public:
  TrajOptProb(){}
  TrajOptProb(int n_steps, ConfigurationPtr rad, CollisionCheckerPtr coll, SQPConfig config);
  ~TrajOptProb() {}
  VarVector GetVarRow(int i) {
    return m_traj_vars.row(i);
  }
  Var& GetVar(int i, int j) {
    return m_traj_vars.at(i,j);
  }
  VarArray& GetVars() {
    return m_traj_vars;
  }
  int GetNumSteps() {return m_traj_vars.rows();}
  int GetNumDOF() {return m_traj_vars.cols();}
  ConfigurationPtr GetRAD() {return m_rad;}
  CollisionCheckerPtr GetCollisionChecker() {return m_collision_checker;}

  inline void setInitTraj(DblVec start, DblVec goal) {
      m_init_traj = trajopt::getStraightLineTrajData(GetNumSteps(), GetNumDOF(), start, goal);
      VarVector vars = GetVarRow(GetNumSteps()-1);
      int n_dof = vars.size();

      for (int j=0; j < n_dof; ++j) {
          //        addLinearConstraint(sco::exprSub(AffExpr(vars[j]), goal_traj[j]), EQ);
          addLinearConstraint(exprSub(AffExpr(m_traj_vars(0,j)), start[j]), EQ);

      }
  }
  inline void setGoalTraj(DblVec goal_traj) {
      VarVector vars = GetVarRow(GetNumSteps()-1);
      int n_dof = vars.size();
      for (int j=0; j < n_dof; ++j) {
//        addLinearConstraint(sco::exprSub(AffExpr(vars[j]), goal_traj[j]), EQ);
          addLinearConstraint(exprSub(AffExpr(m_traj_vars(GetNumSteps()-1,j)), goal_traj[j]), EQ);
      }
  }
  inline void initTraj(DblVec start, DblVec goal){
      setInitTraj(start, goal);
      setGoalTraj(goal);
  }
  TrajArray GetInitTraj() {return m_init_traj;}

  SQPConfig getSQPConfig(){return sqp_config;}

  void setRobotModel(ConfigurationPtr rad){ m_rad = rad; }
  void setCollisionChecker(CollisionCheckerPtr coll){m_collision_checker = coll; }

  friend TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo&, SQPConfig config);

private:
  int n_steps;
  VarArray m_traj_vars;
  ConfigurationPtr m_rad;
  TrajArray m_init_traj;
  CollisionCheckerPtr m_collision_checker;
  SQPConfig sqp_config;
};

struct TRAJOPT_API TrajOptResult {
  OptStatus status;
  vector<string> cost_names, cnt_names;
  vector<double> cost_vals, cnt_viols;
  TrajArray traj;
  TrajOptResult(OptResults& opt, TrajOptProb& prob, OptStatus status);
  TrajOptResult():status(OptStatus::INVALID){}
};

struct BasicInfo  {
  bool start_fixed;
  int n_steps;
  string manip;
  string robot; // optional
  IntVec dofs_fixed; // optional
  void fromJson(const Json::Value& v);
  void fromYaml(const YAML::Node& v);
};

/**
Initialization info read from json
*/
struct InitInfo {
  enum Type {
    STATIONARY,
    GIVEN_TRAJ,
  };
  Type type;
  TrajArray data;

  void fromJson(const Json::Value& v);
  void fromYaml(const YAML::Node& v);
};


struct TRAJOPT_API MakesCost {
};
struct TRAJOPT_API MakesConstraint {
};

/**
When cost or constraint element of JSON doc is read, one of these guys gets constructed to hold the parameters.
Then it later gets converted to a Cost object by the hatch method
*/
struct TRAJOPT_API TermInfo  {

  string name; // xxx is this used?
  TermType term_type;
  virtual void fromJson(const Json::Value& v)=0;
  virtual void fromYaml(const YAML::Node& v)=0;
  virtual void hatch(TrajOptProb& prob) = 0;
  

  static TermInfoPtr fromName(const string& type);

  /**
   * Registers a user-defined TermInfo so you can use your own cost
   * see function RegisterMakers.cpp
   */
  typedef TermInfoPtr (*MakerFunc)(void);
  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~TermInfo() {}
private:
  static std::map<string, MakerFunc> name2maker;
};
 void fromJson(const Json::Value& v, TermInfoPtr&);
 void fromYaml(const YAML::Node& v, TermInfoPtr&);

/**
This object holds all the data that's read from the JSON document
*/
struct TRAJOPT_API ProblemConstructionInfo {
public:
  BasicInfo basic_info;
  vector<TermInfoPtr> cost_infos;
  vector<TermInfoPtr> cnt_infos;
  InitInfo init_info;
  ConfigurationPtr rad;
  CollisionCheckerPtr collision_checker;

  void fromJson(const Value& v);
  void fromYaml(const YAML::Node& v);
};

/**
 \brief pose error

 See trajopt::PoseTermInfo
 */
struct PoseCostInfo : public TermInfo, public MakesCost, public MakesConstraint {
  int timestep;
  Vector3d xyz;
  Vector4d wxyz;
  Vector3d pos_coeffs, rot_coeffs;
   double coeff;
  Link link;
  void fromJson(const Value& v);
  void fromYaml(const YAML::Node& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(PoseCostInfo);
};


/**
  \brief Joint space position cost

  \f{align*}{
  \sum_i c_i (x_i - xtarg_i)^2
  \f}
  where \f$i\f$ indexes over dof and \f$c_i\f$ are coeffs
 */
struct JointPosCostInfo : public TermInfo, public MakesCost {
  DblVec vals, coeffs;
  int timestep;
  void fromJson(const Value& v);
  void fromYaml(const YAML::Node& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointPosCostInfo)
};



/**
 \brief Motion constraint on link

 Constrains the change in position of the link in each timestep to be less than max_displacement
 */
struct CartVelCntInfo : public TermInfo, public MakesConstraint {
  int first_step, last_step;
  Link link;
  double max_displacement;
  void fromJson(const Value& v);
  void fromYaml(const YAML::Node& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(CartVelCntInfo)
};

/**
\brief Joint-space velocity squared

\f{align*}{
  cost = \sum_{t=0}^{T-2} \sum_j c_j (x_{t+1,j} - x_{t,j})^2
\f}
where j indexes over DOF, and \f$c_j\f$ are the coeffs.
*/
struct JointVelCostInfo : public TermInfo, public MakesCost {
  DblVec coeffs;
  void fromJson(const Value& v);
  void fromYaml(const YAML::Node& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointVelCostInfo)
};

struct JointVelConstraintInfo : public TermInfo, public MakesConstraint {
  DblVec vals;
  DblVec m_lb_limits, m_ub_limits;

  int first_step, last_step;
  int m_samples, m_duration, m_dof;
  void fromJson(const Value& v);
  void fromYaml(const YAML::Node& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointVelConstraintInfo)
};

/**
\brief %Collision penalty

Distrete-time penalty:
\f{align*}{
  cost = \sum_{t=0}^{T-1} \sum_{A, B} | distpen_t - sd(A,B) |^+
\f}

Continuous-time penalty: same, except you consider swept-out shaps of robot links. Currently self-collisions are not included.

*/
struct CollisionCostInfo : public TermInfo, public MakesCost {
  /// first_step and last_step are inclusive
  int first_step, last_step;
  /// coeffs.size() = num_timesteps
  DblVec coeffs;
  /// safety margin: contacts with distance < dist_pen are penalized
  DblVec dist_pen;
  bool continuous;
  /// for continuous-time penalty, use swept-shape between timesteps t and t+gap (gap=1 by default)
  int gap;
  void fromJson(const Value& v);
  void fromYaml(const YAML::Node& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(CollisionCostInfo)
};


// TODO: unify with joint position constraint
/**
joint-space position constraint
 */
struct JointConstraintInfo : public TermInfo, public MakesConstraint {
  /// joint values. list of length 1 automatically gets expanded to list of length n_dof
  DblVec vals;
  /// which timestep. default = n_timesteps - 1
  int timestep;
  void fromJson(const Value& v);
  void fromYaml(const YAML::Node& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointConstraintInfo)
};



}
