#include <wrapper/trajopt/TrajoptPlanner.hpp>
#include <wrapper/trajopt/RobotModelWrapper.h>
#include <wrapper/trajopt/FCLCollisionChecker.h>
#include <algorithm>

namespace motion_planners
{

TrajoptPlanner::TrajoptPlanner()
{
}

TrajoptPlanner::~TrajoptPlanner()
{
}

bool TrajoptPlanner::initializePlanner(std::shared_ptr<RobotModel>& robot_model, std::string config_file_path)
{

    m_robot_model_ = robot_model;


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

     Json::Value root = readJsonFile(string(config_file_path));

     m_prb = ConstructProblem(root, robot_model_wrapper, collision_checker_wrapper);

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
    std::cout << "starting updateInitialTrajectory .. . . .. . . . . \n";
    m_opt.setProblem(m_prb);
//    opt.addCallback(boost::bind(&PlotCosts, boost::ref(prob->getCosts()),*prob->GetRAD(), prob->GetVars(), _1));
    DblVec start_traj(m_prb->GetNumDOF());

    for (int i = 0; i < start_traj.size(); i++){
        start_traj.at(i) = start.elements.at(i).position;
    }

//    m_opt.initialize(DblVec(m_prb->GetNumDOF() * m_prb->GetNumSteps(), 0));
//    m_opt.initialize(start_traj);
    m_opt.initialize(trajToDblVec(m_prb->GetInitTraj()));

    std::cout << "ending updateInitialTrajectory .. . . .. . . . . \n";

}

}// end namespace
