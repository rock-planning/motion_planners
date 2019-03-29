#ifndef FCLCOLLISIONCHECKER_H
#define FCLCOLLISIONCHECKER_H

#include <robot_model/RobotModel.hpp>
#include "trajopt/trajoptinterface.h"


using namespace trajopt;
using namespace robot_model;

class FCLCollisionChecker : public CollisionChecker
{

    double m_distance_tolerance;
    std::shared_ptr<RobotModel> m_robot_model_;
    bool m_is_collision_check;

protected:
//    FCLCollisionChecker();
//    FCLCollisionChecker(const FCLCollisionChecker&); // Prevent construction by copying
//    FCLCollisionChecker& operator=(const FCLCollisionChecker&); // Prevent assignment
//    virtual ~FCLCollisionChecker(); // Prevent unwanted destruction
public:
    FCLCollisionChecker(): m_is_collision_check(true) {}
    FCLCollisionChecker(std::shared_ptr<RobotModel> &robot_model);
    virtual ~FCLCollisionChecker();

    void setRobotModel(std::shared_ptr<RobotModel> robot_model){ m_robot_model_ = robot_model; }

    void isCollsionCheckNeeded(bool use_collision){ m_is_collision_check = use_collision; }

    // CollisionChecker interface
    void setDistanceTolerance(float distance);
    double getContactDistance();
    void getContinuousCollisionInfo(const DblVec &startjoints, const DblVec &endjoints, vector<Collision> &collisions);
    void getDiscreteCollisionInfo(vector<Collision> &collisions);
private:
    void transformPointToLocalFrame(const std::string &link_name, const Eigen::Vector3d &point_in, Eigen::Vector3d &point_out);
    void transformNormalToLocalFrame(const std::string &link_name, const Eigen::Vector3d &normal_in, Eigen::Vector3d &normal_out);
    void copyDistanceInfoToCollision(const collision_detection::DistanceInformation &dist, Collision &coll);
};

typedef boost::shared_ptr<FCLCollisionChecker> FCLCollisionCheckerPtr;

#endif // FCLCOLLISIONCHECKER_H
