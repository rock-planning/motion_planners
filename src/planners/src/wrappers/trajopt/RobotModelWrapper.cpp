#include "wrapper/trajopt/RobotModelWrapper.h"
#include "urdf_model/joint.h"

void RobotModelWrapper::DblVecToEigenVectorXd(const DblVec &in, Eigen::VectorXd &out){
    out = Eigen::VectorXd::Map(in.data(), in.size());
}
RobotModelWrapper::RobotModelWrapper(std::shared_ptr<RobotModel> &robot_model):
    m_robot_model(robot_model)
{
    std::cout << "testing if i reach here  .. .  .. . . \n";
    m_planning_group_name_ = robot_model->getPlanningGroupName();
    robot_model->getPlanningGroupJointsName(robot_model->getPlanningGroupName(), m_planning_group_joints_names_);

    std::cout << "yes ....  ..  . . . " << m_planning_group_name_<< " \n";

    std::cout << "testing if i reach here 1 .. .  .. . . \n";
}

void RobotModelWrapper::setRobotModel(std::shared_ptr<RobotModel> robot_model)
{
    m_robot_model = robot_model;
    m_planning_group_name_ = robot_model->getPlanningGroupName();
    robot_model->getPlanningGroupJointsName(robot_model->getPlanningGroupName(), m_planning_group_joints_names_);
}

void RobotModelWrapper::SetDOFValues(const DblVec &dofs) {
    Eigen::VectorXd joint_values;
    DblVecToEigenVectorXd(dofs, joint_values);
    m_robot_model->updateJointGroup(m_planning_group_joints_names_, joint_values);

}

void RobotModelWrapper::GetDOFLimits(DblVec &lower, DblVec &upper) const {
//    lower.resize(m_planning_group_joints_names_.size());
//    upper.resize(m_planning_group_joints_names_.size());

//    for (int i; i < m_planning_group_joints_names_.size(); i++){
//        lower.at(i) = m_robot_model->getRobotState().robot_joints_[m_planning_group_joints_names_[i]].getJointInfo().limits->lower;
//        upper.at(i) = m_robot_model->getRobotState().robot_joints_[m_planning_group_joints_names_[i]].getJointInfo().limits->upper;
//    }
    m_robot_model->getJointLimits(lower, upper);
}
void RobotModelWrapper::GetDOFValues(std::map<std::string, double> &joint_states_map) {

    vector<std::string> joint_names(m_planning_group_joints_names_.size());
    DblVec joint_values(m_planning_group_joints_names_.size());

    GetDOFValues(joint_names, joint_values);

    assert(joint_names.size() == joint_values.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
        joint_states_map[joint_names.at(i)] = joint_values.at(i);
}

void RobotModelWrapper::GetDOFValues(vector<std::string> &joint_names, DblVec &joint_values) {
    joint_names.resize(m_planning_group_joints_names_.size());
    joint_values.resize(m_planning_group_joints_names_.size());

    for (int i; i < m_planning_group_joints_names_.size(); i++){
        joint_names.at(i) = m_planning_group_joints_names_.at(i);
        joint_values.at(i) = m_robot_model->getRobotState().robot_joints_[m_planning_group_joints_names_.at(i)].getJointValue();
    }
}

DblVec RobotModelWrapper::GetDOFValues() {
    vector<std::string> joint_names(m_planning_group_joints_names_.size());
    DblVec joint_values(m_planning_group_joints_names_.size());

    GetDOFValues(joint_names, joint_values);
    return joint_values;
}

int RobotModelWrapper::GetDOF() const {
    return m_planning_group_joints_names_.size();
}

DblMatrix RobotModelWrapper::PositionJacobian(std::string link_name, const Vector3d &pt) const {
//    std::vector< std::pair<std::string,urdf::Joint> > planning_groups_joints;
//    std::string base_link, tip_link;

//    m_robot_model->getPlanningGroupJointinformation(m_planning_group_name_, planning_groups_joints,
//                                                    base_link, link_name);

//    if(planning_groups_joints.size() > 0){
//        tip_link = planning_groups_joints.at(link_name).second.name;
//    }


    std::cout << "Getting position jacobian . . .. . .. . . \n ";

    DblMatrix m /*=  Eigen::MatrixXd::Zero()*/;
    m.resize(3, GetDOF());

    std::cout << "After Getting position jacobian . . .. . .. . . \n ";

    return m;

}

DblMatrix RobotModelWrapper::RotationJacobian(std::string link_name) const {}

void RobotModelWrapper::GetAffectedLinks(std::vector<LinkPtr> &links, bool only_with_geom, vector<std::string> &link_inds)
{

//    std::map<std::string, RobotLink> robot_links_ = m_robot_model->getRobotState().robot_links_;


}

DblVec RobotModelWrapper::RandomDOFValues() {
    std::map<std::string, double> planning_groups_joints_with_random_values;

    m_robot_model->generateRandomJointValue(m_planning_group_name_, planning_groups_joints_with_random_values);

    DblVec random_joint_values(m_planning_group_joints_names_.size());

    for (int i; i < m_planning_group_joints_names_.size(); i++){
        random_joint_values.at(i) = planning_groups_joints_with_random_values[m_planning_group_joints_names_[i]];
    }
    return random_joint_values;
}

geometry::Transform RobotModelWrapper::GetLinkTransformByName(std::string link_name){

    Eigen::Vector3d pos;
    Eigen::Vector4d orn;

    m_robot_model->getLinkTransformByName(link_name, pos, orn);

    return geometry::Transform(geometry::Vector(orn[0], orn[1], orn[2], orn[3]), geometry::Vector(pos[0], pos[1], pos[2]));

}
