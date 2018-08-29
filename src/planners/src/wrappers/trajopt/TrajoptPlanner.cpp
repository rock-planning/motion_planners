#include <wrapper/trajopt/TrajoptPlanner.hpp>

#include <algorithm>
#include "trajopt/TrajoptConfig.h"
#include "wrapper/trajopt/HandleTrajoptConfig.h"

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


TrajoptPlanner::TrajoptPlanner():
    m_robot_model_wrapper(new RobotModelWrapper()),
    m_collision_checker_wrapper(new FCLCollisionChecker()), m_prb(new TrajOptProb)
{

}

TrajoptPlanner::~TrajoptPlanner()
{
}

bool TrajoptPlanner::initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string config_file_path)
{
    std::cout << "starting initializePlanner . . . .. . . . . .. . . \n";

    m_robot_model_ = robot_model;

    m_robot_model_wrapper->setRobotModel(robot_model);
    m_collision_checker_wrapper->setRobotModel(robot_model);

    FCLCollisionCheckerPtr coll(new FCLCollisionChecker(robot_model));
    RobotModelWrapperPtr rob(new RobotModelWrapper(robot_model));

    std::cout << "num of DOF 1 . . . . . . . . ." << rob->GetDOF() << "\n";

    m_prb->setCollisionChecker(coll);
    m_prb->setRobotModel(rob);

    m_prb->init(5);

    std::cout << "num of DOF  2. . . . . . . . ." << m_prb->GetNumDOF() << "\n";


    m_planning_group_name_ = m_robot_model_->getPlanningGroupName();

//    YAML::Node input_config;
//    handle_config::loadConfigFile(config_file_path, input_config);
//    const YAML::Node& stomp_node = input_config["stomp"];
//    const YAML::Node& debug_node = input_config["debug"];

//    stomp_config_ = handle_config::getStompConfig(stomp_node);
//    debug_config_ = handle_config::getDebugConfig(debug_node);


    //get planning grouup joint names.
    m_robot_model_->getPlanningGroupJointsName(m_planning_group_name_, m_planning_group_joints_name_);


     motion_planners::loadConfigFile(config_file_path, m_input_config);

     std::cout << "before calling ConstructProblem . . . .. . . . . .. . . \n";


//     TrajoptConfig config = handle_trajopt_config::getTrajoptConfig(input_config);
//     ProblemModel::constructProblem(config, m_prb);

     std::cout << "after calling ConstructProblem . . . .. . . . . .. . . \n";

     std::cout << "ending initializePlanner . . . .. . . . . .. . . \n";

    return true;
}

bool TrajoptPlanner::solve(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    std::cout << "starting solve method . . . .. . . . . .. . . \n";

//    double tStart = GetClock();
//    m_opt.optimize();
//    std::cout<< "planning time: %.3f\n", GetClock()-tStart;

//    const DblVec& goal_traj = m_opt.x();

//    if(!m_opt.x().size()) {
//        planner_status = PlannerStatus::NO_PATH_FOUND;
//        return false;
//    }

//    const TrajArray& goal_traj  = getTraj(m_opt.x(), m_prb->GetVars());

    m_results = ProblemModel::OptimizeProblem(m_prb);

    const TrajArray& goal_traj  = m_results->traj;

    solution.names.resize(m_planning_group_joints_name_.size());
    solution.elements.resize(m_planning_group_joints_name_.size());


//    vector<double> vec(goal_traj.data(), goal_traj.data() + goal_traj.rows() * goal_traj.cols());
//    std::cout << "result  . . . . .. . . . . . . . ." << std::endl;
//    std::cout << "************************************" << std::endl;

//    for (int i=0; i<m_opt.x().size(); i++)
//        std::cout << m_opt.x().at(i) << std::endl;

//     std::cout << "---------------------------------------------------------" << std::endl;


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


    m_prb = ConstructProblem(m_input_config, m_robot_model_wrapper, m_collision_checker_wrapper);


    std::cout << "starting updateInitialTrajectory .. . . .. . . . . \n";

//    std::cout << "num of DOF  . . . . . . . . ." << m_prb->GetNumDOF() << "\n";

    DblVec start_traj(m_prb->GetNumDOF());
    DblVec goal_traj(m_prb->GetNumDOF());

    for (int i = 0; i < start_traj.size(); i++){
        start_traj.at(i) = start.elements.at(i).position;
        goal_traj.at(i) = goal.elements.at(i).position;
    }


//    std::cout << "before set problem .. . . .. . . . . \n";

//    m_opt.setProblem(m_prb);

//    std::cout << "after set problem.. . . .. . . . . \n";

    std::cout << "Start: " <<start_traj << "\n";
    std::cout << "Goal: " <<goal_traj << "\n";


    std::cout << "before set intial traj .. . . .. . . . . \n";

    m_prb->setInitTraj(start_traj, goal_traj);

    std::cout << "after set intial traj .. . . .. . . . . \n";

//    std::cout << "before intialize .. . . .. . . . . \n";

//    m_opt.initialize(trajToDblVec(m_prb->GetInitTraj()));


//    std::cout << "1 .. . . .. . . . . \n";


    m_prb->setGoalTraj(goal_traj);


//    std::cout << "after intialize .. . . .. . . . . \n";

//    m_opt.initialize(trajopt::getStraightLineTrajData(goal_traj));




    std::cout << "ending updateInitialTrajectory .. . . .. . . . . \n";

}

}// end namespace
