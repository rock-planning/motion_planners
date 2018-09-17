#include "wrapper/trajopt/RobotModelWrapper.h"
#include "urdf_model/joint.h"

typedef Matrix<float, 3, 4> Matrix4f;


template <typename Map>
bool key_compare (Map const &lhs, Map const &rhs) {

    auto pred = [] (decltype(*lhs.begin()) a, decltype(a) b)
    { return a.first == b.first; };

    return lhs.size() == rhs.size()
            && std::equal(lhs.begin(), lhs.end(), rhs.begin(), pred);
}

void skew(Eigen::Vector3d& v, Eigen::Matrix3d* result) {
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
        //        abort();
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
        //        abort();
    }
    for (int col = 0; col < b.cols(); col++) {
        for (int row = 0; row < 3; row++) {
            (*result)(row, col) = a(row, col) - b(row, col);
        }
    }
}

ostream& operator<<(ostream& os, const std::map<std::string, double>& vec) {
    typedef typename std::map<std::string, double>::const_iterator iter_t;
    const iter_t iter_begin = vec.begin();
    const iter_t iter_end   = vec.end();
    os << "\n---------------\n";
    for (iter_t iter = iter_begin; iter != iter_end; ++iter) {
        std::cout << iter->first << " : " << iter->second << "\n";
    }
    os << "\n---------------\n";
    return os;
}

void RobotModelWrapper::DblVecToEigenVectorXd(const DblVec &in, Eigen::VectorXd &out){
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

    for (int i = 0; i < m_planning_group_joints_names_.size(); i++){
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

DblMatrix RobotModelWrapper::PositionJacobian(std::string link_name, const Vector3d &pt) /*const*/ {

    //    std::cout <<"inside Position jacobian   .. .  .. .. " << link_name<< "\n";


    DblMatrix m /*=  Eigen::MatrixXd::Zero()*/;
    m.resize(3, GetDOF());

    std::vector< std::pair<std::string,urdf::Joint> > planning_groups_joints;
    std::string base_link, tip_link;

    m_robot_model->getPlanningGroupJointinformation(m_planning_group_name_, planning_groups_joints,
                                                    base_link, tip_link);

    //    std::cout <<base_link << "==" << tip_link<< "==" << link_name<< "\n";


    //    if(planning_groups_joints.size() > 0){
    //        tip_link = planning_groups_joints.at(link_name).second.name;
    //    }

    KDL::Jacobian  jacobian;

    std::map<std::string, double> joint_states_map;

    m_robot_model->getChainJointState(base_link, link_name, joint_states_map);


    //    std::cout << "joint_states_map size:  " << joint_states_map.empty() << " : "<< joint_states_map.size() << "\n";

    //    std::cout << joint_states_map << "\n";

    m_robot_model->computeJacobain(base_link, link_name, joint_states_map, jacobian);

    KDL::Jacobian  jacobian1 = jacobian;
    KDL::Jacobian  jacobian2 = jacobian;
    KDL::Jacobian  jacobian3 = jacobian;

    m.block(0,0,3, joint_states_map.size()) = jacobian.data.block(0,0,3,  joint_states_map.size());

    DblMatrix m0 /*=  Eigen::MatrixXd::Zero()*/;
    m0.resize(3, GetDOF());

    DblMatrix m1 /*=  Eigen::MatrixXd::Zero()*/;
    m1.resize(3, GetDOF());

    DblMatrix m2 /*=  Eigen::MatrixXd::Zero()*/;
    m2.resize(3, GetDOF());

    DblMatrix m3 /*=  Eigen::MatrixXd::Zero()*/;
    m3.resize(3, GetDOF());

    m0 = m;

//    std::cout <<"before change ref point . .. . . . .. . \n";

//    std::cout << m << "\n";

    KDL::Frame link_frame = m_robot_model->getRobotState().robot_links_[link_name].getLinkFrame();


    KDL::Vector pt_in_local(pt.x(), pt.y(), pt.z());

    //    KDL::Vector pt_in_local(0, 0, 0);


    KDL::Vector pt_in_global = link_frame.Inverse() * pt_in_local;
    KDL::Vector pt_in_global1 = link_frame * pt_in_local;

    //    jacobian.changeRefPoint(KDL::Vector(pt.x(), pt.y(), pt.z()));

    jacobian1.changeRefPoint(pt_in_global);

    jacobian2.changeRefPoint(pt_in_global1);

//    std::cout << "original jacobian . .. . \n"<< jacobian1.data << std::endl;

    m1.block(0,0,3, joint_states_map.size()) = jacobian1.data.block(0,0,3,  joint_states_map.size());

//    std::cout <<"after change ref point global frame 1. .. . . . .. . \n";
//    std::cout << m1 << "\n";

    jacobian2.changeRefPoint(pt_in_local);
    m2.block(0,0,3, joint_states_map.size()) = jacobian2.data.block(0,0,3,  joint_states_map.size());


//    std::cout <<"after change ref point global frame 2. .. . . . .. . \n";
//    std::cout << m2 << "\n";

    jacobian3.changeRefPoint(pt_in_local);



    m3.block(0,0,3, joint_states_map.size()) = jacobian3.data.block(0,0,3,  joint_states_map.size());

//    std::cout <<"after change ref point local frame. .. . . . .. . \n";
//    std::cout << m3 << "\n";



    KDL::Vector pt_rotated = link_frame.M * pt_in_local;

//    std::cout << "local point before .. . .. . " << pt << std::endl;


    Eigen::Vector3d pt_rot(pt_rotated[0], pt_rotated[1], pt_rotated[2]);

//    std::cout << "local point after.. . .. . " << pt_rot << std::endl;


    Eigen::Matrix3d mat_skew;

    skew(pt_rot, &mat_skew);

//    Eigen::Matrix3d M /*=  Eigen::MatrixXd::Zero()*/;

//    for(int i = 0; i < 3; i++){
//        for(int j = 0; j < 3; j++){
//            M(i, j) = link_frame.M(i, j);
//        }
//    }
//    std::cout << "world_rotation_body .  .. . . .. . . \n"<< M << std::endl;

//    std::cout << "skewCrossProduct . . .. . .  . \n"<< mat_skew << std::endl;


    Eigen::Matrix3Xd m5 /*=  Eigen::MatrixXd::Zero()*/;
    m5.resize(3, GetDOF());


    Eigen::Matrix3Xd jac_rot /*=  Eigen::MatrixXd::Zero()*/;
    jac_rot.resize(3, GetDOF());

    jac_rot.block(0,0,3, joint_states_map.size()) = jacobian1.data.block(3,0,3,  joint_states_map.size());

    Eigen::Matrix3Xd jac_rot1 /*=  Eigen::MatrixXd::Zero()*/;
    jac_rot1.resize(3, GetDOF());

    mul(mat_skew, jac_rot, &jac_rot1);

//    std::cout << "jac_rot .  .. . . .. . . \n"<< jac_rot << std::endl;

//    std::cout << "jac_rot1 .  .. . . .. . . \n"<< jac_rot1 << std::endl;

    Eigen::Matrix3Xd jac_new /*=  Eigen::MatrixXd::Zero()*/;
    jac_new.resize(3, GetDOF());

    sub(m0, jac_rot1, &jac_new);

    DblMatrix jac_new1 /*=  Eigen::MatrixXd::Zero()*/;
    jac_new1.resize(3, GetDOF());

    jac_new1 = jac_new;

    return jac_new1;

//    std::cout<< "according to bullet physics . . .. . .  ..  \n";

//    std::cout << jac_new << std::endl;




    //    std::map<std::string, double> joint_states_map1;

    //        KDL::Jacobian  jacobian1;

    //    m_robot_model->getChainJointState(base_link, tip_link, joint_states_map1);

    //     std::cout <<" before compute jaco . . .. . . \n";
    //    m_robot_model->computeJacobain(base_link, tip_link, joint_states_map1, jacobian1);
    //    std::cout <<" after compute jaco . . .. . . \n";

    //    std::cout << jacobian1.data << "\n";

    //    KDL::Jacobian  jacobian1;

    //    std::string base_link1 =base_link, tip_link1= tip_link;


    //    std::map<std::string, double> joint_states_map1;

    //    m_robot_model->getChainJointState(base_link1, tip_link1, joint_states_map1);

    //    std::cout <<" before compute jaco . . .. . . \n";

    //    m_robot_model->computeJacobain(base_link1, tip_link1, joint_states_map, jacobian1);

    //    std::cout <<" after compute jaco . . .. . . \n";

    //        std::cout <<"jacobian1.data . .. . . . .. . \n";

    //        std::cout << jacobian1.data << "\n";

    //        std::cout <<"---------------------------- \n";


//    return m;

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
