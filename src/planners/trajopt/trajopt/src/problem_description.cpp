#include "trajopt/problem_description.hpp"
#include "trajopt/common.hpp"
#include <boost/foreach.hpp>
#include "utils/logging.hpp"
#include "sco/expr_ops.hpp"
#include "sco/expr_op_overloads.hpp"
#include "sco/SQPConfig.h"
#include "trajopt/kinematic_terms.hpp"
#include "trajopt/trajectory_costs.hpp"
#include "trajopt/collision_terms.hpp"
#include "utils/eigen_conversions.hpp"
#include "utils/eigen_slicing.hpp"
#include <boost/algorithm/string.hpp>
#include "sco/optimizers.hpp"
#include <fstream>
#include "yaml-cpp/yaml.h"

using namespace Json;
using namespace std;
using namespace trajopt;
using namespace util;
using namespace yaml_marshal;

namespace {
bool gRegisteredMakers = false;
void ensure_only_members(const Value& v, const char** fields, int nvalid) {
  for (Json::ValueConstIterator it = v.begin(); it != v.end(); ++it) {
    bool valid = false;
    for (int j=0; j < nvalid; ++j) {
      if ( strcmp(it.memberName(), fields[j]) == 0) {
        valid = true;
        break;
      }
    }
    if (!valid) {
      PRINT_AND_THROW( boost::format("invalid field found: %s")%it.memberName());
    }
  }
}

void ensure_only_members(const YAML::Node& v, const char** fields, int nvalid) {
  for (YAML::Node::const_iterator it = v.begin(); it != v.end(); ++it) {
    bool valid = false;
    for (int j=0; j < nvalid; ++j) {
      if ( strcmp(it->first.as<std::string>().c_str(), fields[j]) == 0) {
        valid = true;
        break;
      }
    }
    if (!valid) {
      PRINT_AND_THROW( boost::format("invalid field found: %s")%it->first.as<std::string>());
    }
  }
}

void RegisterMakers() {

  TermInfo::RegisterMaker("pose", &PoseCostInfo::create);
  TermInfo::RegisterMaker("joint_pos", &JointPosCostInfo::create);
  TermInfo::RegisterMaker("joint_vel", &JointVelCostInfo::create);
  TermInfo::RegisterMaker("collision", &CollisionCostInfo::create);

  TermInfo::RegisterMaker("joint", &JointConstraintInfo::create);
  TermInfo::RegisterMaker("cart_vel", &CartVelCntInfo::create);
  TermInfo::RegisterMaker("joint_vel_limits", &JointVelConstraintInfo::create);

  gRegisteredMakers = true;
}

#if 0
BoolVec toMask(const VectorXd& x) {
  BoolVec out(x.size());
  for (int i=0; i < x.size(); ++i) out[i] = (x[i] > 0);
  return out;
}
#endif

bool allClose(const VectorXd& a, const VectorXd& b) {
  return (a-b).array().abs().maxCoeff() < 1e-4;
}

}

namespace Json { //funny thing with two-phase lookup

Json::Value readJsonFile(const std::string& fname) {
  Json::Value root;
  Json::Reader reader;
  std::ifstream fh(fname.c_str());
  bool parse_success = reader.parse(fh, root);
  if (!parse_success) throw std::runtime_error("failed to parse " +fname);
  return root;
}

void fromJson(const Json::Value& v, Vector3d& x) {
  vector<double> vx;
  fromJsonArray(v, vx, 3);
  x = Vector3d(vx[0], vx[1], vx[2]);
}

void fromJson(const Json::Value& v, Vector4d& x) {
  vector<double> vx;
  fromJsonArray(v, vx, 4);
    x = Vector4d(vx[0], vx[1], vx[2], vx[3]);
}
}

namespace YAML{
//void readYamlFile(std::string filename, YAML::Node &config)
//{
//    try
//    {
//        config = YAML::LoadFile(filename);
//    }
//    catch (YAML::ParserException& e)
//    {
//        std::cout << e.what() << "\n";
//    }
//}

void fromYaml(const YAML::Node& v, Vector3d& x) {
  vector<double> vx;
  fromYamlArray(v, vx, 3);
  x = Vector3d(vx[0], vx[1], vx[2]);
}
void fromYaml(const YAML::Node& v, Vector4d& x) {
  vector<double> vx;
  fromYamlArray(v, vx, 4);
    x = Vector4d(vx[0], vx[1], vx[2], vx[3]);
}
}

namespace trajopt {

void readYamlFile(char* filename, YAML::Node &config)
{
    try
    {
        config = YAML::Load(filename);
    }
    catch (YAML::ParserException& e)
    {
        std::cout << e.what() << "\n";
    }
}

SQPConfig getSQPConfig(const Json::Value& root){
    SQPConfig config;
    if (root.isMember("sqp")) {
      Json::Value v = root["sqp"];
      childFromJson(v, config.improve_ratio_threshold_, "improve_ratio_threshold");
      childFromJson(v, config.min_trust_box_size_, "min_trust_box_size");
      childFromJson(v, config.min_approx_improve_, "min_approx_improve");
      childFromJson(v, config.min_approx_improve_frac_, "min_approx_improve_frac");
      childFromJson(v, config.max_iter_, "max_iter");
      childFromJson(v, config.trust_shrink_ratio_, "trust_shrink_ratio");
      childFromJson(v, config.trust_expand_ratio_, "trust_expand_ratio");
      childFromJson(v, config.cnt_tolerance_, "cnt_tolerance");
      childFromJson(v, config.max_merit_coeff_increases_, "max_merit_coeff_increases");
      childFromJson(v, config.merit_coeff_increase_ratio_, "merit_coeff_increase_ratio");
      childFromJson(v, config.merit_error_coeff_, "merit_error_coeff");
      childFromJson(v, config.trust_box_size_, "trust_box_size");
    }
    else {
      PRINT_AND_THROW("missing field: sqp");
    }
    return config;
}

SQPConfig getSQPConfig(const YAML::Node& root){
    SQPConfig config;
    if (root["sqp"]) {
      YAML::Node v = root["sqp"];
      childFromYaml(v, config.improve_ratio_threshold_, "improve_ratio_threshold");
      childFromYaml(v, config.min_trust_box_size_, "min_trust_box_size");
      childFromYaml(v, config.min_approx_improve_, "min_approx_improve");
      childFromYaml(v, config.min_approx_improve_frac_, "min_approx_improve_frac");
      childFromYaml(v, config.max_iter_, "max_iter");
      childFromYaml(v, config.trust_shrink_ratio_, "trust_shrink_ratio");
      childFromYaml(v, config.trust_expand_ratio_, "trust_expand_ratio");
      childFromYaml(v, config.cnt_tolerance_, "cnt_tolerance");
      childFromYaml(v, config.max_merit_coeff_increases_, "max_merit_coeff_increases");
      childFromYaml(v, config.merit_coeff_increase_ratio_, "merit_coeff_increase_ratio");
      childFromYaml(v, config.merit_error_coeff_, "merit_error_coeff");
      childFromYaml(v, config.trust_box_size_, "trust_box_size");
    }
    else {
      PRINT_AND_THROW("missing field: sqp");
    }
    return config;
}

TRAJOPT_API ProblemConstructionInfo* gPCI;

void BasicInfo::fromJson(const Json::Value& v) {
  childFromJson(v, start_fixed, "start_fixed", true);
  childFromJson(v, n_steps, "n_steps");
  childFromJson(v, manip, "manip");
  childFromJson(v, robot, "robot", string(""));
  childFromJson(v, dofs_fixed, "dofs_fixed", IntVec());
  // TODO: optimization parameters, etc?
}

void BasicInfo::fromYaml(const YAML::Node& v) {
  childFromYaml(v, start_fixed, "start_fixed", true);
  childFromYaml(v, n_steps, "no_of_samples");
  childFromYaml(v, manip, "planning_group");
  childFromYaml(v, robot, "robot", string(""));
  childFromYaml(v, dofs_fixed, "dofs_fixed", IntVec());
  // TODO: optimization parameters, etc?
}


bool gReadingCosts=false, gReadingConstraints=false;
void fromJson(const Json::Value& v, TermInfoPtr& term) {
  string type;
  childFromJson(v, type, "type");
  LOG_DEBUG("reading term: %s", type.c_str());
  term = TermInfo::fromName(type);
  if (gReadingCosts) {
    if (!term) PRINT_AND_THROW( boost::format("failed to construct cost named %s")%type );
    if (!dynamic_cast<MakesCost*>(term.get())) PRINT_AND_THROW( boost::format("%s is only a constraint, but you listed it as a cost")%type) ;
    term->term_type = TT_COST;
  }
  else if (gReadingConstraints) {
    if (!term) PRINT_AND_THROW( boost::format("failed to construct constraint named %s")%type );
    if (!dynamic_cast<MakesConstraint*>(term.get())) PRINT_AND_THROW( boost::format("%s is only a cost, but you listed it as a constraint")%type);
    term->term_type = TT_CNT;
  }
  else assert(0 && "shouldnt happen");
  term->fromJson(v);
  childFromJson(v, term->name, "name", type);
}

void fromYaml(const YAML::Node& v, TermInfoPtr& term) {
  string type;
  childFromYaml(v, type, "type");
  LOG_DEBUG("reading term: %s", type.c_str());
  term = TermInfo::fromName(type);
  if (gReadingCosts) {
    if (!term) PRINT_AND_THROW( boost::format("failed to construct cost named %s")%type );
    if (!dynamic_cast<MakesCost*>(term.get())) PRINT_AND_THROW( boost::format("%s is only a constraint, but you listed it as a cost")%type) ;
    term->term_type = TT_COST;
  }
  else if (gReadingConstraints) {
    if (!term) PRINT_AND_THROW( boost::format("failed to construct constraint named %s")%type );
    if (!dynamic_cast<MakesConstraint*>(term.get())) PRINT_AND_THROW( boost::format("%s is only a cost, but you listed it as a constraint")%type);
    term->term_type = TT_CNT;
  }
  else assert(0 && "shouldnt happen");
  term->fromYaml(v);
  childFromYaml(v, term->name, "name", type);
}

std::map<string, TermInfo::MakerFunc> TermInfo::name2maker;
void TermInfo::RegisterMaker(const std::string& type, MakerFunc f) {
  name2maker[type] = f;
}

TermInfoPtr TermInfo::fromName(const string& type) {
  if (!gRegisteredMakers) RegisterMakers();
  if (name2maker.find(type) != name2maker.end()) {
    return (*name2maker[type])();
  }
  else {
    PRINT_ERROR(boost::format("There is no cost of type %s\n")% type.c_str());
    return TermInfoPtr();
  }
}

TrajArray getStationaryTrajData(DblVec startpoint){
    TrajArray data;
    int n_steps = gPCI->basic_info.n_steps;
     data = toVectorXd(startpoint).transpose().replicate(n_steps, 1);
    return data;
}

TrajArray getStraightLineTrajData(int n_steps, int n_dof, DblVec startpoint, DblVec endpoint){
    TrajArray data;
    if (endpoint.size() != n_dof) {
        PRINT_AND_THROW(boost::format("wrong number of dof values in initialization. expected %i got %j")%n_dof%endpoint.size());
    }
    data = TrajArray(n_steps, n_dof);

    for (int idof = 0; idof < n_dof; ++idof) {
        data.col(idof) = VectorXd::LinSpaced(n_steps, startpoint[idof], endpoint[idof]);
    }
    return data;
}

void InitInfo::fromJson(const Json::Value& v) {
    string type_str;
    childFromJson(v, type_str, "type");
    int n_steps = gPCI->basic_info.n_steps;
    int n_dof = gPCI->rad->getDOF();
    if (type_str == "stationary") {
        data = getStationaryTrajData(gPCI->rad->getDOFValues());
    }
    else if (type_str == "given_traj") {
        FAIL_IF_FALSE(v.isMember("data"));
        const Value& vdata = v["data"];
        if (vdata.size() != n_steps) {
            PRINT_AND_THROW("given initialization traj has wrong length");
        }
        data.resize(n_steps, n_dof);
        for (int i=0; i < n_steps; ++i) {
            DblVec row;
            fromJsonArray(vdata[i], row, n_dof);
            data.row(i) = toVectorXd(row);
        }
    }
    else if (type_str == "straight_line") {
        FAIL_IF_FALSE(v.isMember("endpoint"));
        DblVec endpoint;
        childFromJson(v, endpoint, "endpoint");
        if (endpoint.size() != n_dof) {
            PRINT_AND_THROW(boost::format("wrong number of dof values in initialization. expected %i got %j")%n_dof%endpoint.size());
        }
        data = getStraightLineTrajData(n_steps, n_dof, gPCI->rad->getDOFValues(), endpoint);
    }
}

void InitInfo::fromYaml(const YAML::Node& v) {
  string type_str;
  childFromYaml(v, type_str, "type");
  int n_steps = gPCI->basic_info.n_steps;
  int n_dof = gPCI->rad->getDOF();

  if (type_str == "stationary") {
    data = getStationaryTrajData(gPCI->rad->getDOFValues());
  }
  else if (type_str == "given_traj") {
    FAIL_IF_FALSE(v["data"]);
    const YAML::Node& vdata = v["data"];
    if (vdata.size() != n_steps) {
      PRINT_AND_THROW("given initialization traj has wrong length");
    }
    data.resize(n_steps, n_dof);
    for (int i=0; i < n_steps; ++i) {
      DblVec row;
      fromYamlArray(vdata[i], row, n_dof);
      data.row(i) = toVectorXd(row);
    }
  }
  else if (type_str == "straight_line") {
    FAIL_IF_FALSE(v["endpoint"]);
    DblVec endpoint;
    childFromYaml(v, endpoint, "endpoint");
    if (endpoint.size() != n_dof) {
      PRINT_AND_THROW(boost::format("wrong number of dof values in initialization. expected %i got %j")%n_dof%endpoint.size());
    }
    data = getStraightLineTrajData(n_steps, n_dof, gPCI->rad->getDOFValues(), endpoint);
  }

}

void ProblemConstructionInfo::fromJson(const Value& v) {
  childFromJson(v, basic_info, "basic_info");
  gPCI = this;
  gReadingCosts=true;
  gReadingConstraints=false;
  if (v.isMember("costs")) fromJsonArray(v["costs"], cost_infos);
  gReadingCosts=false;
  gReadingConstraints=true;
  if (v.isMember("constraints")) fromJsonArray(v["constraints"], cnt_infos);
  gReadingConstraints=false;

  childFromJson(v, init_info, "init_info");
  gPCI = NULL;

}

void ProblemConstructionInfo::fromYaml(const YAML::Node& v) {
  childFromYaml(v, basic_info, "basic_info");
  gPCI = this;
  gReadingCosts=true;
  gReadingConstraints=false;
  if (v["costs"]) fromYamlArray(v["costs"], cost_infos);
  gReadingCosts=false;
  gReadingConstraints=true;
  if (v["constraints"]) fromYamlArray(v["constraints"], cnt_infos);
  gReadingConstraints=false;

  if(v["init_info"])
    childFromYaml(v, init_info, "init_info");
  gPCI = NULL;

}

TrajOptResult::TrajOptResult(OptResults& opt, TrajOptProb& prob, OptStatus status) :
  cost_vals(opt.cost_vals),
  cnt_viols(opt.cnt_viols), status(status) {
  BOOST_FOREACH(const CostPtr& cost, prob.getCosts()) {
    cost_names.push_back(cost->name());
  }
  BOOST_FOREACH(const ConstraintPtr& cnt, prob.getConstraints()) {
    cnt_names.push_back(cnt->name());
  }
  traj = getTraj(opt.x, prob.GetVars());
}

TrajOptResultPtr OptimizeProblem(TrajOptProbPtr prob, SQPConfig config) {

//  BasicTrustRegionSQP opt(prob, config);
    BasicTrustRegionSQP opt(prob, prob->getSQPConfig());

//  opt.sqp_config.max_iter_ = 40;
//  opt.sqp_config.min_approx_improve_frac_ = .001;
//  opt.sqp_config.improve_ratio_threshold_ = .2;
//  opt.sqp_config.merit_error_coeff_ = 20;
//  opt.sqp_config.trust_box_size_ = 0.5;


  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  OptStatus status = opt.optimize();
  if (status != OptStatus::INVALID)
    return TrajOptResultPtr(new TrajOptResult(opt.results(), *prob, status));
  else
      return TrajOptResultPtr(new TrajOptResult());
}

TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo& pci, SQPConfig config) {

  const BasicInfo& bi = pci.basic_info;
  int n_steps = bi.n_steps;

  TrajOptProbPtr prob(new TrajOptProb(n_steps, pci.rad, pci.collision_checker, config));

//  int n_dof = prob->m_rad->GetDOF();

//  DblVec cur_dofvals = prob->m_rad->GetDOFValues();

//  if (bi.start_fixed) {
//    if (pci.init_info.data.rows() > 0 && cur_dofvals.size() > 0 && !allClose(toVectorXd(cur_dofvals), pci.init_info.data.row(0))) {
//      PRINT_AND_THROW( "robot dof values don't match initialization. I don't know what you want me to use for the dof values");
//    }
//    for (int j=0; j < n_dof; ++j) {
//      prob->addLinearConstraint(exprSub(AffExpr(prob->m_traj_vars(0,j)), cur_dofvals[j]), EQ);
//    }
//  }

//  if (!bi.dofs_fixed.empty()) {
//    BOOST_FOREACH(const int& dof_ind, bi.dofs_fixed) {
//      for (int i=1; i < prob->GetNumSteps(); ++i) {
//        prob->addLinearConstraint(exprSub(AffExpr(prob->m_traj_vars(i,dof_ind)), AffExpr(prob->m_traj_vars(0,dof_ind))), EQ);
//      }
//    }
//  }

  BOOST_FOREACH(const TermInfoPtr& ci, pci.cost_infos) {
    ci->hatch(*prob);
  }
  BOOST_FOREACH(const TermInfoPtr& ci, pci.cnt_infos) {
    ci->hatch(*prob);
  }
//  prob->SetInitTraj(pci.init_info.data);

  return prob;
}

TrajOptProbPtr ConstructProblem(const Json::Value& root, ConfigurationPtr rad, CollisionCheckerPtr coll) {
  ProblemConstructionInfo pci;
  pci.rad = rad;
  pci.collision_checker = coll;
  pci.fromJson(root);
  SQPConfig config = getSQPConfig(root);
  return ConstructProblem(pci, config);
}

TrajOptProbPtr ConstructProblem(const YAML::Node& root, ConfigurationPtr rad, CollisionCheckerPtr coll) {
  ProblemConstructionInfo pci;
  pci.rad = rad;
  pci.collision_checker = coll;
  pci.fromYaml(root);
  SQPConfig config = getSQPConfig(root);
  return ConstructProblem(pci, config);
}

TrajOptProb::TrajOptProb(int n_steps, ConfigurationPtr rad, CollisionCheckerPtr coll, SQPConfig config) : n_steps(n_steps), m_rad(rad), m_collision_checker(coll), sqp_config(config) {

    DblVec lower, upper;
    m_rad->getDOFLimits(lower, upper);
    int n_dof = m_rad->getDOF();

    vector<double> vlower, vupper;
    vector<string> names;
    for (int i=0; i < n_steps; ++i) {
      vlower.insert(vlower.end(), lower.data(), lower.data()+lower.size());
      vupper.insert(vupper.end(), upper.data(), upper.data()+upper.size());
      for (unsigned j=0; j < n_dof; ++j) {
        names.push_back( (boost::format("j_%i_%i")%i%j).str() );
      }
    }
    VarVector trajvarvec = createVariables(names, vlower, vupper);
    m_traj_vars = VarArray(n_steps, n_dof, trajvarvec.data());
}


void PoseCostInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, timestep, "timestep", gPCI->basic_info.n_steps-1);
  childFromJson(params, xyz,"xyz");
  childFromJson(params, wxyz,"wxyz");
  childFromJson(params, pos_coeffs,"pos_coeffs", (Vector3d)Vector3d::Ones());
  childFromJson(params, rot_coeffs,"rot_coeffs", (Vector3d)Vector3d::Ones());
  childFromJson(params, link, "link");
  if (!gPCI->rad->checkIfLinkExists(link)) {
    PRINT_AND_THROW(boost::format("invalid link name: %s")%link);
  }

  const char* all_fields[] = {"timestep", "xyz", "wxyz", "pos_coeffs", "rot_coeffs","link"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));

}

void PoseCostInfo::fromYaml(const YAML::Node& v) {
  FAIL_IF_FALSE(v["params"]);
  const YAML::Node& params = v["params"];
  childFromYaml(params, timestep, "timestep", gPCI->basic_info.n_steps-1);
  childFromYaml(params, xyz,"xyz");
  childFromYaml(params, wxyz,"wxyz");
  childFromYaml(params, pos_coeffs,"pos_coeffs", (Vector3d)Vector3d::Ones());
  childFromYaml(params, rot_coeffs,"rot_coeffs", (Vector3d)Vector3d::Ones());

  childFromYaml(params, link, "link");
  if (!gPCI->rad->checkIfLinkExists(link)) {
    PRINT_AND_THROW(boost::format("invalid link name: %s")%link);
  }

  const char* all_fields[] = {"timestep", "xyz", "wxyz", "pos_coeffs", "rot_coeffs","link"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));

}

void PoseCostInfo::hatch(TrajOptProb& prob) {
  VectorOfVectorPtr f(new CartPoseErrCalculator(toRaveTransform(wxyz, xyz), prob.GetRAD(), link));
  if (term_type == TT_COST) {
    prob.addCost(CostPtr(new CostFromErrFunc(f, prob.GetVarRow(timestep), concat(rot_coeffs, pos_coeffs), ABS, name)));
  }
  else if (term_type == TT_CNT) {
    prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f, prob.GetVarRow(timestep), concat(rot_coeffs, pos_coeffs), EQ, name)));
  }
}


void JointPosCostInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  int n_steps = gPCI->basic_info.n_steps;
  const Value& params = v["params"];
  childFromJson(params, vals, "vals");
  childFromJson(params, coeffs, "coeffs");
  if (coeffs.size() == 1) coeffs = DblVec(n_steps, coeffs[0]);

  int n_dof = gPCI->rad->getDOF();
  if (vals.size() != n_dof) {
    PRINT_AND_THROW( boost::format("wrong number of dof vals. expected %i got %i")%n_dof%vals.size());
  }
  childFromJson(params, timestep, "timestep", gPCI->basic_info.n_steps-1);

  const char* all_fields[] = {"vals", "coeffs", "timestep"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));
}

void JointPosCostInfo::fromYaml(const YAML::Node& v) {
  FAIL_IF_FALSE(v["params"]);
  int n_steps = gPCI->basic_info.n_steps;
  const YAML::Node& params = v["params"];
  childFromYaml(params, vals, "vals");
  childFromYaml(params, coeffs, "coeffs");
  if (coeffs.size() == 1) coeffs = DblVec(n_steps, coeffs[0]);

  int n_dof = gPCI->rad->getDOF();
  if (vals.size() != n_dof) {
    PRINT_AND_THROW( boost::format("wrong number of dof vals. expected %i got %i")%n_dof%vals.size());
  }
  childFromYaml(params, timestep, "timestep", gPCI->basic_info.n_steps-1);

  const char* all_fields[] = {"vals", "coeffs", "timestep"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));
}


void JointPosCostInfo::hatch(TrajOptProb& prob) {
  prob.addCost(CostPtr(new JointPosCost(prob.GetVarRow(timestep), toVectorXd(vals), toVectorXd(coeffs))));
  prob.getCosts().back()->setName(name);
}


void CartVelCntInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, first_step, "first_step");
  childFromJson(params, last_step, "last_step");
  childFromJson(params, max_displacement,"max_displacement");

  FAIL_IF_FALSE((first_step >= 0) && (first_step <= gPCI->basic_info.n_steps-1) && (first_step < last_step));
  FAIL_IF_FALSE((last_step > 0) && (last_step <= gPCI->basic_info.n_steps-1));

  childFromJson(params, link, "link");
  if (!gPCI->rad->checkIfLinkExists(link)) {
    PRINT_AND_THROW(boost::format("invalid link name: %s")%link);
  }

  const char* all_fields[] = {"first_step", "last_step", "max_displacement","link"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));
}

void CartVelCntInfo::fromYaml(const YAML::Node& v) {
  FAIL_IF_FALSE(v["params"]);
  const YAML::Node& params = v["params"];
  childFromYaml(params, first_step, "first_step");
  childFromYaml(params, last_step, "last_step");
  childFromYaml(params, max_displacement,"max_displacement");

  FAIL_IF_FALSE((first_step >= 0) && (first_step <= gPCI->basic_info.n_steps-1) && (first_step < last_step));
  FAIL_IF_FALSE((last_step > 0) && (last_step <= gPCI->basic_info.n_steps-1));

  childFromYaml(params, link, "link");
  if (!gPCI->rad->checkIfLinkExists(link)) {
    PRINT_AND_THROW(boost::format("invalid link name: %s")%link);
  }
  const char* all_fields[] = {"first_step", "last_step", "max_displacement","link"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));


}

void CartVelCntInfo::hatch(TrajOptProb& prob) {
  for (int iStep = first_step; iStep < last_step; ++iStep) {
    prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(
      VectorOfVectorPtr(new CartVelCalculator(prob.GetRAD(), link, max_displacement)),
       MatrixOfVectorPtr(new CartVelJacCalculator(prob.GetRAD(), link, max_displacement)),
      concat(prob.GetVarRow(iStep), prob.GetVarRow(iStep+1)), VectorXd::Ones(0), INEQ, "CartVel")));
  }
}

void JointVelCostInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  childFromJson(params, coeffs,"coeffs");
  int n_dof = gPCI->rad->getDOF();
  if (coeffs.size() == 1) coeffs = DblVec(n_dof, coeffs[0]);
  else if (coeffs.size() != n_dof) {
    PRINT_AND_THROW( boost::format("wrong number of coeffs. expected %i got %i")%n_dof%coeffs.size());
  }

  const char* all_fields[] = {"coeffs"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));


}

void JointVelCostInfo::fromYaml(const YAML::Node& v) {
  FAIL_IF_FALSE(v["params"]);
  const YAML::Node& params = v["params"];

  childFromYaml(params, coeffs,"coeffs");
  int n_dof = gPCI->rad->getDOF();
  if (coeffs.size() == 1) coeffs = DblVec(n_dof, coeffs[0]);
  else if (coeffs.size() != n_dof) {
    PRINT_AND_THROW( boost::format("wrong number of coeffs. expected %i got %i")%n_dof%coeffs.size());
  }

  const char* all_fields[] = {"coeffs"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));


}

void JointVelCostInfo::hatch(TrajOptProb& prob) {
  prob.addCost(CostPtr(new JointVelCost(prob.GetVars(), toVectorXd(coeffs))));
  prob.getCosts().back()->setName(name);
}


void JointVelConstraintInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  int n_steps = gPCI->basic_info.n_steps;
  int n_dof = gPCI->rad->getDOF();
  childFromJson(params, vals, "vals");
  childFromJson(params, first_step, "first_step", 0);
  childFromJson(params, last_step, "last_step", n_steps-1);
  FAIL_IF_FALSE(vals.size() == n_dof);
  FAIL_IF_FALSE((first_step >= 0) && (first_step < n_steps));
  FAIL_IF_FALSE((last_step >= first_step) && (last_step < n_steps));

  const char* all_fields[] = {"vals", "first_step", "last_step"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));

}

void JointVelConstraintInfo::fromYaml(const YAML::Node& v) {
  FAIL_IF_FALSE(v["params"]);
  const YAML::Node& params = v["params"];

  int n_steps = gPCI->basic_info.n_steps;
  int n_dof = gPCI->rad->getDOF();

  m_dof = gPCI->rad->getDOF();

  gPCI->rad->getDOFLimits(m_lb_limits, m_ub_limits);

  m_samples = gPCI->basic_info.n_steps;

  childFromYaml(params, m_duration, "duration", 10);

  childFromYaml(params, vals, "vals");
  childFromYaml(params, first_step, "first_step", 0);
  childFromYaml(params, last_step, "last_step", n_steps-1);
  if (vals.size() == 1) vals = DblVec(n_dof, vals[0]);
  FAIL_IF_FALSE(vals.size() == n_dof);
  FAIL_IF_FALSE((first_step >= 0) && (first_step < n_steps));
  FAIL_IF_FALSE((last_step >= first_step) && (last_step < n_steps));

  const char* all_fields[] = {"vals", "first_step", "last_step", "duration"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));

}

void JointVelConstraintInfo::hatch(TrajOptProb& prob) {
  for (int i = first_step; i <= last_step-1; ++i) {
    for (int j=0; j < vals.size(); ++j)  {
      AffExpr vel = prob.GetVar(i+1,j) -  prob.GetVar(i,j);
      vel.constant *= (10 / 20);
      prob.addLinearConstraint(vel - vals[j], INEQ);
      prob.addLinearConstraint(-vel - vals[j], INEQ);
    }
  }
}

void CollisionCostInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  int n_steps = gPCI->basic_info.n_steps;
  childFromJson(params, continuous, "continuous", true);
  childFromJson(params, first_step, "first_step", 0);
  childFromJson(params, last_step, "last_step", n_steps-1);
  childFromJson(params, gap, "gap", 1);
  FAIL_IF_FALSE( gap >= 0 );
  FAIL_IF_FALSE((first_step >= 0) && (first_step < n_steps));
  FAIL_IF_FALSE((last_step >= first_step) && (last_step < n_steps));
  childFromJson(params, coeffs, "coeffs");
  int n_terms = last_step - first_step + 1;
  if (coeffs.size() == 1) coeffs = DblVec(n_terms, coeffs[0]);
  else if (coeffs.size() != n_terms) {
    PRINT_AND_THROW (boost::format("wrong size: coeffs. expected %i got %i")%n_terms%coeffs.size());
  }
  childFromJson(params, dist_pen,"dist_pen");
  if (dist_pen.size() == 1) dist_pen = DblVec(n_terms, dist_pen[0]);
  else if (dist_pen.size() != n_terms) {
    PRINT_AND_THROW(boost::format("wrong size: dist_pen. expected %i got %i")%n_terms%dist_pen.size());
  }

  const char* all_fields[] = {"continuous", "first_step", "last_step", "gap", "coeffs", "dist_pen"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));
}

void CollisionCostInfo::fromYaml(const YAML::Node& v) {
  FAIL_IF_FALSE(v["params"]);
  const YAML::Node& params = v["params"];

  int n_steps = gPCI->basic_info.n_steps;
  childFromYaml(params, continuous, "continuous", true);
  childFromYaml(params, first_step, "first_step", 0);
  childFromYaml(params, last_step, "last_step", n_steps-1);
  childFromYaml(params, gap, "gap", 1);
  FAIL_IF_FALSE( gap >= 0 );
  FAIL_IF_FALSE((first_step >= 0) && (first_step < n_steps));
  FAIL_IF_FALSE((last_step >= first_step) && (last_step < n_steps));
  childFromYaml(params, coeffs, "coeffs");
  int n_terms = last_step - first_step + 1;
  if (coeffs.size() == 1) coeffs = DblVec(n_terms, coeffs[0]);
  else if (coeffs.size() != n_terms) {
    PRINT_AND_THROW (boost::format("wrong size: coeffs. expected %i got %i")%n_terms%coeffs.size());
  }
  childFromYaml(params, dist_pen,"dist_pen");
  if (dist_pen.size() == 1) dist_pen = DblVec(n_terms, dist_pen[0]);
  else if (dist_pen.size() != n_terms) {
    PRINT_AND_THROW(boost::format("wrong size: dist_pen. expected %i got %i")%n_terms%dist_pen.size());
  }

  const char* all_fields[] = {"continuous", "first_step", "last_step", "gap", "coeffs", "dist_pen"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));
}

void CollisionCostInfo::hatch(TrajOptProb& prob) {
  if (term_type == TT_COST) {
    if (continuous) {
      for (int i=first_step; i <= last_step - gap; ++i) {
        prob.addCost(CostPtr(new CollisionCost(dist_pen[i-first_step], coeffs[i-first_step], prob.GetRAD(), prob.GetCollisionChecker(), prob.GetVarRow(i), prob.GetVarRow(i+gap))));
        prob.getCosts().back()->setName( (boost::format("%s_%i")%name%i).str() );
      }
    }
    else {
      for (int i=first_step; i <= last_step; ++i) {
        prob.addCost(CostPtr(new CollisionCost(dist_pen[i-first_step], coeffs[i-first_step], prob.GetRAD(), prob.GetCollisionChecker(), prob.GetVarRow(i))));
        prob.getCosts().back()->setName( (boost::format("%s_%i")%name%i).str() );
      }
    }
  }
  else { // ALMOST COPIED
    if (continuous) {
      for (int i=first_step; i < last_step; ++i) {
        prob.addIneqConstraint(ConstraintPtr(new CollisionConstraint(dist_pen[i-first_step], coeffs[i-first_step], prob.GetRAD(), prob.GetCollisionChecker(), prob.GetVarRow(i), prob.GetVarRow(i+1))));
        prob.getIneqConstraints().back()->setName( (boost::format("%s_%i")%name%i).str() );
      }
    }
    else {
      for (int i=first_step; i <= last_step; ++i) {
        prob.addIneqConstraint(ConstraintPtr(new CollisionConstraint(dist_pen[i-first_step], coeffs[i-first_step], prob.GetRAD(), prob.GetCollisionChecker(), prob.GetVarRow(i))));
        prob.getIneqConstraints().back()->setName( (boost::format("%s_%i")%name%i).str() );
      }
    }
  }
  prob.GetCollisionChecker()->setDistanceTolerance(*std::max_element(dist_pen.begin(), dist_pen.end()) + .04);
}


void JointConstraintInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, vals, "vals");

  int n_dof = gPCI->rad->getDOF();
  if (vals.size() != n_dof) {
    PRINT_AND_THROW( boost::format("wrong number of dof vals. expected %i got %i")%n_dof%vals.size());
  }
  childFromJson(params, timestep, "timestep", gPCI->basic_info.n_steps-1);

  const char* all_fields[] = {"vals", "timestep"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));

}

void JointConstraintInfo::fromYaml(const YAML::Node& v) {
  FAIL_IF_FALSE(v["params"]);
  const YAML::Node& params = v["params"];
  childFromYaml(params, vals, "vals");

  int n_dof = gPCI->rad->getDOF();
  if (vals.size() != n_dof) {
    PRINT_AND_THROW( boost::format("wrong number of dof vals. expected %i got %i")%n_dof%vals.size());
  }
  childFromYaml(params, timestep, "timestep", gPCI->basic_info.n_steps-1);

  const char* all_fields[] = {"vals", "timestep"};
  ensure_only_members(params, all_fields, sizeof(all_fields)/sizeof(char*));

}

void JointConstraintInfo::hatch(TrajOptProb& prob) {
  VarVector vars = prob.GetVarRow(timestep);
  int n_dof = vars.size();
  for (int j=0; j < n_dof; ++j) {
    prob.addLinearConstraint(exprSub(AffExpr(vars[j]), vals[j]), EQ);
  }
}


}

