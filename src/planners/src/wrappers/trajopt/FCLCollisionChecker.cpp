#include "wrapper/trajopt/FCLCollisionChecker.h"

ostream& operator<<(ostream& os, const collision_detection::DistanceInformation& dist) {

    os<<"**********DistanceInformation*************** \n"
    << "object 1: " << dist.object1 << " \nobject2:  " << dist.object2 << " \ndist: " << dist.distance<< "\n"
    << "point on O1: \n" << dist.nearest_points.at(0) << " \npoint on O2: \n" << dist.nearest_points.at(1) << "\n"
    << "normal: \n" << dist.contact_normal << " \nmin dist: \n" << dist.min_distance << "\nunit normal \n" << dist.unit_normal<< "\n"
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

void FCLCollisionChecker::SetContactDistance(float distance)
{

    m_contact_tolerance = distance;

}

double FCLCollisionChecker::GetContactDistance()
{

}

void FCLCollisionChecker::GetContinuousCollisionInfo(const DblVec &startjoints, const DblVec &endjoints, vector<Collision> &collisions, double distance_tolerance/*=0.05*/)
{

    std::cout << "**************** starting GetContinuousCollisionInfo****************\n";
//    std::vector<collision_detection::DistanceInformation> distance_info;

//    m_robot_model_->getSelfDistanceInfo(distance_info, distance_tolerance);

//    collisions.resize(distance_info.size());
//    for(int i=0; i<distance_info.size(); i++){
//        Collision &coll = collisions.at(i);

//        collision_detection::DistanceInformation &dist = distance_info.at(i);
////        std::cout << dist << "\n";

//        coll.distance = dist.distance;
//        //            coll.normalB2A = dist.;
//        coll.ptA = dist.nearest_points.at(0);
//        coll.ptB = dist.nearest_points.at(1);
//        coll.time = 0;
//        coll.linkA = dist.object1;
//        coll.linkB = dist.object2;

//        coll.normalB2A = dist.contact_normal;
//        coll.linkB = dist.object2;


////        std::cout << coll << "\n";

//    }


//    std::vector<collision_detection::ContactInformation> contact_info;

//    bool is_collided = m_robot_model_->getSelfCollisionInfo(contact_info);

//    if(is_collided){
//        collisions.resize(contact_info.size());
//        for(int i=0; i<contact_info.size(); i++){
//            Collision &coll = collisions.at(i);

//            collision_detection::ContactInformation &con = contact_info.at(i);

////            std::cout << con << "\n";

//            coll.distance = con.penetration_depth;
//            coll.normalB2A = con.contact_normal / con.contact_normal.norm();
//            coll.ptA = con.contact_position;
//            coll.ptB = con.contact_position;
//            coll.time = 0;
//            coll.linkA = con.object1;
//            coll.linkB = con.object2;
////            std::cout << coll << "\n";

//        }
//    }


//    std::cout << "------------------------------------------------------\n";


//    std::vector<collision_detection::ContactInformation> contact_info;

//    bool is_collided = m_robot_model_->getSelfCollisionInfo(contact_info);

//    if(is_collided){
//        collisions.resize(contact_info.size());
//        for(int i=0; i<contact_info.size(); i++){
//            Collision &coll = collisions.at(i);

//            collision_detection::ContactInformation &con = contact_info.at(i);
//            std::cout << "Collision between . . . ." << con.object1 << " and " << con.object2 << "\n";

//             std::cout << con << "\n";

////            coll.distance = con.penetration_depth;
////            coll.normalB2A = con.contact_normal;
////            coll.ptA = con.contact_position;
////            coll.ptB = con.contact_position;
////            coll.time = 0;
////            coll.linkA = con.object1;
////            coll.linkB = con.object2;
////            std::cout << coll << "\n";

//        }
//    }
//    else
    {
        collisions.clear();
        std::vector<collision_detection::DistanceInformation> distance_info;
        std::cout << "GetContinuousCollisionInfo: before getSelfDistanceInfo ****************\n";
        m_robot_model_->getSelfDistanceInfo(distance_info, m_contact_tolerance);
        std::cout << "GetContinuousCollisionInfo: after getSelfDistanceInfo ****************\n";
//        collisions.resize(distance_info.size());
        for(int i=0; i<distance_info.size(); i++){
//            Collision &coll = collisions.at(i);
            Collision coll;
            collision_detection::DistanceInformation &dist = distance_info.at(i);
//            if(dist.distance < 0 || true){
//            if(dist.distance < distance_tolerance){
//                std::cout << "distance between . . . ." << dist.object1 << " and " << dist.object2 << " : " << dist.distance << "\n";

//                std::cout << dist << "\n";
//            }

            if(dist.distance > 0 && dist.distance < distance_tolerance)
            {
                coll.distance = dist.distance;
                coll.ptA = dist.nearest_points.at(0);
                coll.ptB = dist.nearest_points.at(1);
                coll.time = 0;
                coll.linkA = dist.object1;
                coll.linkB = dist.object2;
                coll.normalB2A = dist.unit_normal;
                collisions.push_back(coll);
//                std::cout << coll << "\n";
            }
            else if (dist.distance <= 0)
            {
                std::vector<collision_detection::ContactInformation> contact_info;
                std::cout << "GetContinuousCollisionInfo: before getSelfCollisionInfo ****************\n";
                bool is_collided = m_robot_model_->getSelfCollisionInfo(contact_info);
                std::cout << "GetContinuousCollisionInfo: after getSelfCollisionInfo ****************\n";
                if(is_collided){
                    collisions.resize(contact_info.size());
                    for(int i=0; i<contact_info.size(); i++){
                        Collision &coll = collisions.at(i);
                        collision_detection::ContactInformation &con = contact_info.at(i);
//                        std::cout << "Collision between . . . ." << con.object1 << " and " << con.object2 << " : " << - con.penetration_depth  << "\n";
//                         std::cout << con << "\n";
                        coll.distance = - con.penetration_depth;
                        coll.normalB2A = con.contact_normal;
                        coll.ptA = con.contact_position;
                        coll.ptB = con.contact_position;
                        coll.time = 0;
                        coll.linkA = con.object1;
                        coll.linkB = con.object2;
            //            std::cout << coll << "\n";
                        collisions.push_back(coll);

                    }
                }
            }

        }

    }

    std::cout << "**************** ending GetContinuousCollisionInfo****************\n";
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
            coll.linkA = con.object1;
            coll.linkB = con.object2;

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
            coll.linkA = dist.object1;
            coll.linkB = dist.object2;
        }

    }
}

