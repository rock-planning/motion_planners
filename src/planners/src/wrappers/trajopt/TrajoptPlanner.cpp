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
    return true;
}

bool TrajoptPlanner::solve(base::JointsTrajectory &solution, PlannerStatus &planner_status)
{
    m_results = OptimizeProblem(m_prb);
    if(m_results->status != OptStatus::INVALID){
        const TrajArray& goal_traj  = m_results->traj;
//        std::cout << "goal_traj : \n" << goal_traj << std::endl;
        solution.names.resize(m_planning_group_joints_name_.size());
        solution.elements.resize(m_planning_group_joints_name_.size());
        for(int col = 0; col < goal_traj.cols(); col++)
        {
            solution.names.at(col) = m_planning_group_joints_name_.at(col);
            solution.elements.at(col).resize(goal_traj.col(col).size());
            for( int row = 0; row < goal_traj.rows(); row++)
            {
                solution.elements.at(col).at(row).position =  goal_traj.coeff(row, col);
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
//    std::cout << "Start: " <<start_traj << "\n";
//    std::cout << "Goal: " <<goal_traj << "\n";
    m_prb->initTraj(start_traj, goal_traj);

}

}// end namespace
