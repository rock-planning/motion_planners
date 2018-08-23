#include <wrapper/trajopt/TrajoptPlanner.hpp>

#include <algorithm>
#include "sco/expr_ops.hpp"

namespace motion_planners
{

template <typename TElem>
ostream& operator<<(ostream& os, const vector<TElem>& vec) {
    typedef typename vector<TElem>::const_iterator iter_t;
    const iter_t iter_begin = vec.begin();
    const iter_t iter_end   = vec.end();
    os << "[";
    for (iter_t iter = iter_begin; iter != iter_end; ++iter) {
        std::cout << ((iter != iter_begin) ? "," : "") << *iter;
    }
    os << "]";
    return os;
}


TrajoptPlanner::TrajoptPlanner()
//    m_robot_model_wrapper(new RobotModelWrapper()),
//    m_collision_checker_wrapper(new FCLCollisionChecker())
{

}

TrajoptPlanner::~TrajoptPlanner()
{
}

bool TrajoptPlanner::initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string config_file_path)
{
    std::cout << "starting initializePlanner . . . .. . . . . .. . . \n";

    m_robot_model_ = robot_model;

//    m_robot_model_wrapper->setRobotModel(robot_model);
//    m_collision_checker_wrapper->setRobotModel(robot_model);

    m_planning_group_name_ = m_robot_model_->getPlanningGroupName();

//    YAML::Node input_config;
//    handle_config::loadConfigFile(config_file_path, input_config);
//    const YAML::Node& stomp_node = input_config["stomp"];
//    const YAML::Node& debug_node = input_config["debug"];

//    stomp_config_ = handle_config::getStompConfig(stomp_node);
//    debug_config_ = handle_config::getDebugConfig(debug_node);


    //get planning grouup joint names.
    m_robot_model_->getPlanningGroupJointsName(m_planning_group_name_, m_planning_group_joints_name_);


    RobotModelWrapperPtr robot_model_wrapper(new RobotModelWrapper(m_robot_model_));

    FCLCollisionCheckerPtr collision_checker_wrapper(new FCLCollisionChecker(m_robot_model_));

     YAML::Node input_config;

     motion_planners::loadConfigFile(config_file_path, input_config);
//     m_prb(new TrajOptProb(5, robot_model_wrapper, collision_checker_wrapper));
//       TrajOptProbPtr prob(new TrajOptProb(5, robot_model_wrapper, collision_checker_wrapper));
//       m_prb = prob;

     std::cout << "before calling ConstructProblem . . . .. . . . . .. . . \n";

     m_prb = ConstructProblem(input_config, robot_model_wrapper, collision_checker_wrapper);

     std::cout << "after calling ConstructProblem . . . .. . . . . .. . . \n";

     std::cout << "ending initializePlanner . . . .. . . . . .. . . \n";

    return true;
}

bool TrajoptPlanner::solve(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    std::cout << "starting solve method . . . .. . . . . .. . . \n";

    double tStart = GetClock();
    m_opt.optimize();
    std::cout<< "planning time: %.3f\n", GetClock()-tStart;

//    const DblVec& goal_traj = m_opt.x();
    const TrajArray& goal_traj  = getTraj(m_opt.x(), m_prb->GetVars());

    solution.names.resize(m_planning_group_joints_name_.size());
    solution.elements.resize(m_planning_group_joints_name_.size());


//    vector<double> vec(goal_traj.data(), goal_traj.data() + goal_traj.rows() * goal_traj.cols());
    std::cout << "result  . . . . .. . . . . . . . ." << std::endl;
    std::cout << "************************************" << std::endl;

    for (int i=0; i<m_opt.x().size(); i++)
        std::cout << m_opt.x().at(i) << std::endl;

     std::cout << "---------------------------------------------------------" << std::endl;


    for(int d = 0; d < goal_traj.cols(); d++)
    {
        solution.names.at(d) = m_planning_group_joints_name_.at(d);

        solution.elements.at(d).resize(goal_traj.col(d).size());


//        solution.elements.at(d).resize(2);
        for( int i = 0; i < goal_traj.rows(); i++)
        {
             solution.elements.at(d).at(i).position =  goal_traj.coeff(i, d);
        }
    }
    std::cout << "goal_traj.rows()  . . . . .. . . . . . . . ." << goal_traj.rows() << std::endl;
    std::cout << "goal_traj.cols() . . . . .. . . . . . . . ." << goal_traj.cols() << std::endl;
    std::cout << "m_opt.x() . . . . .. . . . . . . . ." << m_opt.x().size() << std::endl;



    std::cout << "ending solve method . . . .. . . . . .. . . \n";
}


void TrajoptPlanner::updateInitialTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal, PlannerStatus &planner_status)
{

//    double increment = 0;

//    for (int d=0; d < stomp_config_.num_dimensions_; ++d)
//    {
//    initial_trajectory_[d].head(stomp::TRAJECTORY_PADDING) 	= 1.0 * start.elements.at(d).position * Eigen::VectorXd::Ones(stomp::TRAJECTORY_PADDING);
//    initial_trajectory_[d].tail(stomp::TRAJECTORY_PADDING) 	= 1.0 * goal.elements.at(d).position  * Eigen::VectorXd::Ones(stomp::TRAJECTORY_PADDING);

//    increment = (goal.elements.at(d).position - start.elements.at(d).position)/(stomp_config_.num_time_steps_ - 1);

//    for (int i=0; i < stomp_config_.num_time_steps_; i++)
//        initial_trajectory_[d](stomp::TRAJECTORY_PADDING+i) = start.elements.at(d).position+(i*increment);
//    }

    std::cout << "starting updateInitialTrajectory .. . . .. . . . . \n";

    DblVec start_traj(m_prb->GetNumDOF());
    DblVec goal_traj(m_prb->GetNumDOF());

    for (int i = 0; i < start_traj.size(); i++){
        start_traj.at(i) = start.elements.at(i).position;
        goal_traj.at(i) = goal.elements.at(i).position;
    }


    std::cout << "before set problem .. . . .. . . . . \n";

    m_opt.setProblem(m_prb);

    std::cout << "after set problem.. . . .. . . . . \n";

    std::cout << "Start: " <<start_traj << "\n";
    std::cout << "Goal: " <<goal_traj << "\n";

//    m_opt.initialize(DblVec(m_prb->GetNumDOF() * m_prb->GetNumSteps(), 0));
//    m_opt.initialize(start_traj);

    std::cout << "before set intial traj .. . . .. . . . . \n";

//    m_prb->SetInitTraj(trajopt::getStraightLineTrajData(goal_traj));

    m_prb->SetInitTraj(trajopt::getStraightLineTrajData1(5, 7, start_traj, goal_traj));

    std::cout << "after set intial traj .. . . .. . . . . \n";

    std::cout << "before intialize .. . . .. . . . . \n";

    m_opt.initialize(trajToDblVec(m_prb->GetInitTraj()));


    std::cout << "1 .. . . .. . . . . \n";

    VarVector vars = m_prb->GetVarRow(4);

    std::cout << "2 .. . . .. . . . . \n";

    int n_dof = vars.size();


    std::cout << "3 .. . . .. . . . . \n";

    for (int j=0; j < n_dof; ++j) {
        std::cout << "4 .. . . .. . . . . \n";

      m_prb->addLinearConstraint(sco::exprSub(AffExpr(vars[j]), goal_traj[j]), EQ);
      std::cout << "5 .. . . .. . . . . \n";

    }


    std::cout << "after intialize .. . . .. . . . . \n";

//    m_opt.initialize(trajopt::getStraightLineTrajData(goal_traj));



    std::cout << "ending updateInitialTrajectory .. . . .. . . . . \n";

}

}// end namespace
