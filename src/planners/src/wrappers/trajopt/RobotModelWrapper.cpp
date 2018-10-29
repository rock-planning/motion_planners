#include "wrapper/trajopt/RobotModelWrapper.h"
#include "urdf_model/joint.h"

typedef Matrix<float, 3, 4> Matrix4f;


template <typename TElem>
ostream& operator<<(ostream& os, const std::vector<TElem>& vec) {
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

template <typename Map>
bool key_compare (Map const &lhs, Map const &rhs) {

    auto pred = [] (decltype(*lhs.begin()) a, decltype(a) b)
    { return a.first == b.first; };

    return lhs.size() == rhs.size()
            && std::equal(lhs.begin(), lhs.end(), rhs.begin(), pred);
}

void skew(Eigen::Vector3d& v, Eigen::Matrix3d* result) {
    /*
    Skew-symmetric matrix:
    A^T = −A.

    x=[a b c]' is a 3*1 vector, and its 3*3 skew symmetric matrix is
    X=[0   -c   b ;
         c    0  -a ;
        -b    a   0  ]
*/
    (*result)(0, 0) = 0.0;
    (*result)(0, 1) = -v(2);
    (*result)(0, 2) = v(1);
    (*result)(1, 0) = v(2);
    (*result)(1, 1) = 0.0;
    (*result)(1, 2) = -v(0);
    (*result)(2, 0) = -v(1);
    (*result)(2, 1) = v(0);
    (*result)(2, 2) = 0.0;
}

void mul(const Eigen::Matrix3d &a, const Eigen::Matrix3Xd &b, Eigen::Matrix3Xd *result) {
    if (b.cols() != result->cols()) {
        std::cerr<<"size missmatch. b.cols()= " << static_cast<int>(b.cols()) << "result->cols()= " << static_cast<int>(result->cols()) << std::endl;
    }

    for (int col = 0; col < b.cols(); col++) {
        const double x = a(0,0)*b(0,col)+a(0,1)*b(1,col)+a(0,2)*b(2,col);
        const double y = a(1,0)*b(0,col)+a(1,1)*b(1,col)+a(1,2)*b(2,col);
        const double z = a(2,0)*b(0,col)+a(2,1)*b(1,col)+a(2,2)*b(2,col);
        (*result)(0, col) = x;
        (*result)(1, col) = y;
        (*result)(2, col) = z;
    }
}

void sub(const Eigen::Matrix3Xd &a, const Eigen::Matrix3Xd &b, Eigen::Matrix3Xd *result) {
    if (a.cols() != b.cols()) {
        std::cerr<<"size missmatch. a.cols()= "<< static_cast<int>(a.cols()) <<"b.cols()= " << static_cast<int>(b.cols());
    }
    for (int col = 0; col < b.cols(); col++) {
        for (int row = 0; row < 3; row++) {
            (*result)(row, col) = a(row, col) - b(row, col);
        }
    }
}

void RobotModelWrapper::dblVecToEigenVectorXd(const DblVec &in, Eigen::VectorXd &out){
    out = Eigen::VectorXd::Map(in.data(), in.size());
}
RobotModelWrapper::RobotModelWrapper(std::shared_ptr<RobotModel> &robot_model):
    m_robot_model(robot_model)
{
    m_planning_group_name_ = robot_model->getPlanningGroupName();
    robot_model->getPlanningGroupJointsName(robot_model->getPlanningGroupName(), m_planning_group_joints_names_);
}

void RobotModelWrapper::setRobotModel(std::shared_ptr<RobotModel> robot_model)
{
    m_robot_model = robot_model;
    m_planning_group_name_ = robot_model->getPlanningGroupName();
    robot_model->getPlanningGroupJointsName(robot_model->getPlanningGroupName(), m_planning_group_joints_names_);
}

void RobotModelWrapper::setDOFValues(const DblVec &dofs) {
    Eigen::VectorXd joint_values;
    dblVecToEigenVectorXd(dofs, joint_values);
    m_robot_model->updateJointGroup(m_planning_group_joints_names_, joint_values);

}

void RobotModelWrapper::getDOFLimits(DblVec &lower, DblVec &upper) const {
    m_robot_model->getJointLimits(lower, upper);
}
void RobotModelWrapper::getDOFValues(std::map<std::string, double> &joint_states_map) {
    std::vector<std::string> joint_names(m_planning_group_joints_names_.size());
    DblVec joint_values(m_planning_group_joints_names_.size());
    getDOFValues(joint_names, joint_values);
    assert(joint_names.size() == joint_values.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
        joint_states_map[joint_names.at(i)] = joint_values.at(i);
}

void RobotModelWrapper::getDOFValues(vector<std::string> &joint_names, DblVec &joint_values) {
    joint_names.resize(m_planning_group_joints_names_.size());
    joint_values.resize(m_planning_group_joints_names_.size());

    for (int i = 0; i < m_planning_group_joints_names_.size(); i++){
        joint_names.at(i) = m_planning_group_joints_names_.at(i);
        joint_values.at(i) = m_robot_model->getRobotState().robot_joints_[m_planning_group_joints_names_.at(i)].getJointValue();
    }
}

DblVec RobotModelWrapper::getDOFValues() {
    vector<std::string> joint_names(m_planning_group_joints_names_.size());
    DblVec joint_values(m_planning_group_joints_names_.size());

    getDOFValues(joint_names, joint_values);
    return joint_values;
}

int RobotModelWrapper::getDOF() const {
    return m_planning_group_joints_names_.size();
}

DblMatrix RobotModelWrapper::getPositionJacobian(std::string link_name, const Vector3d &pt) /*const*/ {


    DblMatrix m /*=  Eigen::MatrixXd::Zero()*/;
    m.resize(3, getDOF());
    m *= 0.000;

    std::vector< std::pair<std::string,urdf::Joint> > planning_groups_joints;
    std::string base_link, tip_link;

    m_robot_model->getPlanningGroupJointinformation(m_planning_group_name_, planning_groups_joints,
                                                    base_link, tip_link);

    KDL::Jacobian  jacobian;

    std::map<std::string, double> joint_states_map;

    m_robot_model->getChainJointState(base_link, link_name, joint_states_map);
    m_robot_model->computeJacobain(base_link, link_name, joint_states_map, jacobian);
//    jacobian.changeRefPoint(KDL::Vector(pt.x(), pt.y(), pt.z()));

    m.block(0,0,3, joint_states_map.size()) = jacobian.data.block(0,0,3,  joint_states_map.size());

    KDL::Frame link_frame = m_robot_model->getRobotState().robot_links_[link_name].getLinkFrame();

    KDL::Vector pt_in_local(pt.x(), pt.y(), pt.z());

    KDL::Vector pt_rotated = link_frame.M * pt_in_local;

    Eigen::Vector3d pt_rot(pt_rotated[0], pt_rotated[1], pt_rotated[2]);

    Eigen::Matrix3d mat_skew;

    skew(pt_rot, &mat_skew);

    Eigen::Matrix3Xd jac_rot /*=  Eigen::MatrixXd::Zero()*/;
    jac_rot.resize(3, getDOF());

    jac_rot *= 0.00;

    jac_rot.block(0,0,3, joint_states_map.size()) = jacobian.data.block(3,0,3,  joint_states_map.size());

    Eigen::Matrix3Xd jac_rot1 /*=  Eigen::MatrixXd::Zero()*/;
    jac_rot1.resize(3, getDOF());

    jac_rot1 *= 0.00;

    mul(mat_skew, jac_rot, &jac_rot1);

    Eigen::Matrix3Xd jac_new /*=  Eigen::MatrixXd::Zero()*/;
    jac_new.resize(3, getDOF());

    sub(m, jac_rot1, &jac_new);

    DblMatrix jac_new1 /*=  Eigen::MatrixXd::Zero()*/;
    jac_new1.resize(3, getDOF());

    jac_new1 = jac_new;

    return jac_new1;

}

DblMatrix RobotModelWrapper::getRotationJacobian(std::string link_name) const {}

DblVec RobotModelWrapper::setRandomDOFValues() {
    std::map<std::string, double> planning_groups_joints_with_random_values;

    m_robot_model->generateRandomJointValue(m_planning_group_name_, planning_groups_joints_with_random_values);

    DblVec random_joint_values(m_planning_group_joints_names_.size());

    for (int i; i < m_planning_group_joints_names_.size(); i++){
        random_joint_values.at(i) = planning_groups_joints_with_random_values[m_planning_group_joints_names_[i]];
    }
    return random_joint_values;
}

geometry::Transform RobotModelWrapper::getLinkTransformByName(std::string link_name){

    Eigen::Vector3d pos;
    Eigen::Vector4d orn;

    m_robot_model->getLinkTransformByName(link_name, pos, orn);

    return geometry::Transform(geometry::Vector(orn[0], orn[1], orn[2], orn[3]), geometry::Vector(pos[0], pos[1], pos[2]));

}

bool RobotModelWrapper::checkIfLinkExists(const std::string link_name)
{
//    if ( m_robot_model->getRobotState().robot_links_.find(link_name) != m_robot_model->getRobotState().robot_links_.end() ) {
    if (m_robot_model->getRobotState().robot_links_.count(link_name)){
        return true;
    }

    return false;
}
