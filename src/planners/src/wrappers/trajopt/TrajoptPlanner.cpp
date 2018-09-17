#include <wrapper/trajopt/TrajoptPlanner.hpp>

#include <algorithm>

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

    m_robot_model_ = robot_model;
    m_robot_model_wrapper->setRobotModel(robot_model);
    m_collision_checker_wrapper->setRobotModel(robot_model);

    m_planning_group_name_ = m_robot_model_->getPlanningGroupName();
    m_robot_model_->getPlanningGroupJointsName(m_planning_group_name_, m_planning_group_joints_name_);
    motion_planners::loadConfigFile(config_file_path, m_input_config);


    std::string link_name = motion_planners::getValue<std::string>(m_input_config["jacobian"], "link_name"); ;

    std::vector<double> pt1(3);
    pt1 = motion_planners::getValue<std::vector<double>>(m_input_config["jacobian"], "point");

    Eigen::Vector3d pt(pt1.at(0), pt1.at(1), pt1.at(2));

     DblVec init = m_robot_model_wrapper->GetDOFValues();
    std::cout << "initial state: "<< init << std::endl;

    std::vector<double> state(m_robot_model_wrapper->GetDOF());
    state = motion_planners::getValue<std::vector<double>>(m_input_config["jacobian"], "state");
    m_robot_model_wrapper->SetDOFValues(state);

    std::cout << "set state: " << m_robot_model_wrapper->GetDOFValues() << std::endl;

    std::cout << "jocobian link_name : " << link_name << std::endl;

    std::cout << "jocobian point : \n" << pt << std::endl;


    DblMatrix m = m_robot_model_wrapper->PositionJacobian(link_name, pt);

    std::cout << "jocobian : \n" << m << std::endl;

    m_robot_model_wrapper->SetDOFValues(init);

//    vector<Collision> collisions;
//    m_collision_checker_wrapper->GetContinuousCollisionInfo(DblVec(), DblVec(), collisions);


     return true;
}

bool TrajoptPlanner::solve(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{

    m_results = OptimizeProblem(m_prb);

    if(m_results->status != OptStatus::INVALID){
        const TrajArray& goal_traj  = m_results->traj;
        solution.names.resize(m_planning_group_joints_name_.size());
        solution.elements.resize(m_planning_group_joints_name_.size());

        for(int d = 0; d < goal_traj.cols(); d++)
        {
            solution.names.at(d) = m_planning_group_joints_name_.at(d);

            solution.elements.at(d).resize(goal_traj.col(d).size());
            for( int i = 0; i < goal_traj.rows(); i++)
            {
                solution.elements.at(d).at(i).position =  goal_traj.coeff(i, d);
            }
        }
        return true;
    }
    else{
        planner_status.statuscode = PlannerStatus::NO_PATH_FOUND;
        return false;
    }
//    std::cout << "goal_traj.rows()  . . . . .. . . . . . . . ." << goal_traj.rows() << std::endl;
//    std::cout << "goal_traj.cols() . . . . .. . . . . . . . ." << goal_traj.cols() << std::endl;
//    std::cout << "m_opt.x() . . . . .. . . . . . . . ." << m_opt.x().size() << std::endl;

    return false;
}


void TrajoptPlanner::updateInitialTrajectory(const base::samples::Joints &start, const base::samples::Joints &goal, PlannerStatus &planner_status)
{


    m_prb = ConstructProblem(m_input_config, m_robot_model_wrapper, m_collision_checker_wrapper);

    DblVec start_traj(m_prb->GetNumDOF());
    DblVec goal_traj(m_prb->GetNumDOF());

    for (int i = 0; i < start_traj.size(); i++){
        start_traj.at(i) = start.elements.at(i).position;
        goal_traj.at(i) = goal.elements.at(i).position;
    }

    std::cout << "Start: " <<start_traj << "\n";
    std::cout << "Goal: " <<goal_traj << "\n";

    m_prb->initTraj(start_traj, goal_traj);

}

}// end namespace
