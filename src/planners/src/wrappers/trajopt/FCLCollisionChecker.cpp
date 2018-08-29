#include "wrapper/trajopt/FCLCollisionChecker.h"

FCLCollisionChecker::FCLCollisionChecker(std::shared_ptr<RobotModel> &robot_model):
    m_robot_model_(robot_model)
{

}

FCLCollisionChecker::~FCLCollisionChecker() {}

void FCLCollisionChecker::SetContactDistance(float distance)
{

}

double FCLCollisionChecker::GetContactDistance()
{

}

void FCLCollisionChecker::GetContinuousCollisionInfo(const DblVec &startjoints, const DblVec &endjoints, vector<Collision> &collisions)
{


}

void FCLCollisionChecker::GetDiscreteCollisionInfo(vector<Collision> &collisions)
{

    std::vector<collision_detection::ContactInformation> contact_info;
    bool is_collided = m_robot_model_->getSelfCollisionInfo(contact_info);
    if(is_collided){
        collisions.resize(contact_info.size());
        for(int i=0; i<contact_info.size(); i++){
            Collision &coll = collisions.at(i);
            collision_detection::ContactInformation &con = contact_info.at(i);
            coll.distance = con.penetration_depth;
            coll.normalB2A = con.contact_normal;
            coll.ptA = con.contact_position;
            coll.ptB = con.contact_position;
            coll.time = 0;
            //        coll.linkA;
            //        coll.linkB;

        }
    }else{
        std::vector<collision_detection::DistanceInformation> distance_info;
        m_robot_model_->getSelfDistanceInfo(distance_info);
        collisions.resize(distance_info.size());
        for(int i=0; i<distance_info.size(); i++){
            Collision &coll = collisions.at(i);
            collision_detection::DistanceInformation &dist = distance_info.at(i);
            coll.distance = dist.distance;
            //            coll.normalB2A = dist.;
            coll.ptA = dist.nearest_points.at(0);
            coll.ptB = dist.nearest_points.at(1);
            coll.time = 0;
            //        coll.linkA;
            //        coll.linkB;
        }

    }
}

