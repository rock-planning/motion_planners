//#include <typeinfo>
#include "wrapper/trajopt/FCLCollisionChecker.h"


ostream& operator<<(ostream& os, const collision_detection::DistanceInformation& dist) {

    os<<"**********DistanceInformation*************** \n"
    << "object 1: " << dist.object1 << " \nobject2:  " << dist.object2 << " \ndist: " << dist.min_distance<< "\n"
    << "point on O1: \n" << dist.nearest_points.at(0) << " \npoint on O2: \n" << dist.nearest_points.at(1) << "\n"
    << "normal: \n" << dist.contact_normal << " \nmin dist: \n" << dist.min_distance << "\n"
    <<"---------------------------------------- \n";
    return os;
}

void FCLCollisionChecker::copyDistanceInfoToCollision(const collision_detection::DistanceInformation &dist, Collision &coll){
    coll.distance = dist.min_distance;
    coll.ptA = dist.nearest_points.at(0);
    coll.ptB = dist.nearest_points.at(1);
    coll.linkA = dist.object1;
    coll.linkB = dist.object2;
    coll.normalB2A = dist.contact_normal;
}

void FCLCollisionChecker::transformPointToLocalFrame(const std::string &link_name, const Eigen::Vector3d &point_in, Eigen::Vector3d &point_out){
    // transform point to local frame only for robot links
    if (m_robot_model_->getRobotState().robot_links_.count(link_name)){
        KDL::Frame link_frame = m_robot_model_->getRobotState().robot_links_[link_name].getLinkFrame();
        KDL::Vector point(point_in.x(), point_in.y(), point_in.z());
        point = link_frame.Inverse() * point;
        point_out = Eigen::Vector3d(point.x(), point.y(), point.z());
    }
}

void FCLCollisionChecker::transformNormalToLocalFrame(const std::string &link_name, const Eigen::Vector3d &normal_in, Eigen::Vector3d &normal_out){

    KDL::Frame link_frame = m_robot_model_->getRobotState().robot_links_[link_name].getLinkFrame().Inverse();
    Eigen::Matrix3d rot;
    for(int i = 0; i<3; i++){
        for(int j = 0; j<3; j++){
            rot(i, j) = link_frame.M(i, j);
        }
    }
    Eigen::Matrix4d trans_mat;
    trans_mat.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    trans_mat.block<3,3>(0,0) = rot;
    trans_mat.block<3,1>(0,3) = Eigen::Vector3d(link_frame.p[0], link_frame.p[1], link_frame.p[2]);

    Eigen::Vector4d normal(normal_in.x(), normal_in.y(), normal_in.z(), 1);
    normal = trans_mat.transpose() * normal;
    normal_out = Eigen::Vector3d(normal[0], normal[1], normal[2]);
}


FCLCollisionChecker::FCLCollisionChecker(std::shared_ptr<RobotModel> &robot_model):
    m_robot_model_(robot_model), m_is_collision_check(true)
{

}

FCLCollisionChecker::~FCLCollisionChecker() {}

void FCLCollisionChecker::setDistanceTolerance(float distance)
{

    m_distance_tolerance = distance;

}

double FCLCollisionChecker::getContactDistance()
{

}

void FCLCollisionChecker::getContinuousCollisionInfo(const DblVec &startjoints, const DblVec &endjoints, vector<Collision> &collisions)
{

    getDiscreteCollisionInfo(collisions);

}

void FCLCollisionChecker::getDiscreteCollisionInfo(vector<Collision> &collisions)
{
/*    
    // This version works with 189f59ade1ca7b7b5c83292ea15c0f5978be79e2 "Master" version - 20191113
    std::vector<collision_detection::DistanceInformation> distance_info;
    m_robot_model_->getRobotDistanceToCollisionInfo(distance_info);
    for(int i=0; i<distance_info.size(); i++)
    {
        Collision coll;
        collision_detection::DistanceInformation &dist = distance_info.at(i);
        if((!m_is_collision_check || dist.min_distance > 0) && dist.min_distance < m_distance_tolerance)
        {
            transformPointToLocalFrame(dist.object1, dist.nearest_points.at(0), dist.nearest_points.at(0));
            transformPointToLocalFrame(dist.object2, dist.nearest_points.at(1), dist.nearest_points.at(1));
            //        transformNormalToLocalFrame(dist.object1, dist.contact_normal, dist.contact_normal);
            dist.contact_normal = dist.nearest_points.at(0) - dist.nearest_points.at(1);
            dist.contact_normal /= dist.contact_normal.norm();

           copyDistanceInfoToCollision(dist, coll);
           collisions.push_back(coll);
        }
//        if(dist.min_distance < m_distance_tolerance){
//            std::cout << "distance between . . . ." << dist.object1 << " and " << dist.object2 << " : " << dist.min_distance << "\n";
//            std::cout << dist << "\n";
//        }
    }

    if (m_is_collision_check)
    {
        std::vector<collision_detection::DistanceInformation> contact_info;
        bool not_collided = false;
        not_collided = m_robot_model_->getRobotCollisionInfo(contact_info);
        if(!not_collided){
            collisions.resize(contact_info.size());
            for(int i=0; i<contact_info.size(); i++){
                Collision coll;
                collision_detection::DistanceInformation &con = contact_info.at(i);
                transformPointToLocalFrame(con.object1, con.nearest_points.at(0), con.nearest_points.at(0));
                transformPointToLocalFrame(con.object1, con.nearest_points.at(1), con.nearest_points.at(1));
                transformNormalToLocalFrame(con.object1, con.contact_normal, con.contact_normal);

//                std::cout << "Collision between . . . ." << con.object1 << " and " << con.object2 << " : " << - con.min_distance  << "\n";
//                std::cout << con << "\n";

                copyDistanceInfoToCollision(con, coll);
                collisions.push_back(coll);
            }
        }
    }*/
}
