#include "wrapper/trajopt/FCLCollisionChecker.h"

ostream& operator<<(ostream& os, const collision_detection::DistanceInformation& dist) {

    os<<"**********DistanceInformation*************** \n"
    << "object 1: " << dist.object1 << " \nobject2:  " << dist.object2 << " \ndist: " << dist.min_distance<< "\n"
    << "point on O1: \n" << dist.nearest_points.at(0) << " \npoint on O2: \n" << dist.nearest_points.at(1) << "\n"
    << "normal: \n" << dist.contact_normal << " \nmin dist: \n" << dist.min_distance << "\n"
    <<"---------------------------------------- \n";
    return os;
}


ostream& operator<<(ostream& os, const collision_detection::ContactInformation& cont) {

    os<<"**********ContactInformation*************** \n"
    << "object 1: " << cont.object1 << " \nobject2:  " << cont.object2  << " \npenetration_depth: " << cont.penetration_depth << "\n"
    << "point on O1: \n" << cont.contact_position<< " \nnormal: \n" << cont.contact_normal<< "\n"
    << " \nunit normal: \n" << (cont.contact_normal /  cont.contact_normal.norm())<< "\n"

    <<"---------------------------------------- \n";

    return os;
}

FCLCollisionChecker::FCLCollisionChecker(std::shared_ptr<RobotModel> &robot_model):
    m_robot_model_(robot_model)
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

    collisions.clear();
    std::vector<collision_detection::DistanceInformation> distance_info;
    m_robot_model_->getSelfDistanceInfo(distance_info, m_distance_tolerance);

    for(int i=0; i<distance_info.size(); i++){
        Collision coll;
        collision_detection::DistanceInformation &dist = distance_info.at(i);

        if(dist.min_distance < m_distance_tolerance){
            std::cout << "distance between . . . ." << dist.object1 << " and " << dist.object2 << " : " << dist.min_distance << "\n";
            std::cout << dist << "\n";
        }

        if(dist.min_distance > 0 && dist.min_distance < m_distance_tolerance)
        {
            coll.distance = dist.min_distance;
            coll.ptA = dist.nearest_points.at(0);
            coll.ptB = dist.nearest_points.at(1);
            coll.linkA = dist.object1;
            coll.linkB = dist.object2;
            coll.normalB2A = dist.contact_normal;
            collisions.push_back(coll);
            //                std::cout << coll << "\n";
        }
        else if (dist.min_distance <= 0)
        {
            std::vector<collision_detection::ContactInformation> contact_info;
            bool is_collided = m_robot_model_->getSelfCollisionInfo(contact_info);
            if(is_collided){
                collisions.resize(contact_info.size());
                for(int i=0; i<contact_info.size(); i++){
                    Collision &coll = collisions.at(i);
                    collision_detection::ContactInformation &con = contact_info.at(i);
                    std::cout << "Collision between . . . ." << con.object1 << " and " << con.object2 << " : " << - con.penetration_depth  << "\n";
                    std::cout << con << "\n";
                    coll.distance = - con.penetration_depth;
                    coll.normalB2A = con.contact_normal;
                    coll.ptA = con.contact_position;
                    coll.ptB = con.contact_position;
                    coll.time = 0;
                    coll.linkA = con.object1;
                    coll.linkB = con.object2;
                    collisions.push_back(coll);
                    //            std::cout << coll << "\n";

                }
            }
        }

    }
}

