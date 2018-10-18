#ifndef FCLCOLLISIONCHECKER_H
#define FCLCOLLISIONCHECKER_H

#include "trajopt/trajoptinterface.h"
#include "RobotModel.hpp"

using namespace trajopt;
using namespace motion_planners;

class FCLCollisionChecker : public CollisionChecker
{

    double m_distance_tolerance;
    std::shared_ptr<RobotModel> m_robot_model_;

protected:
//    FCLCollisionChecker();
//    FCLCollisionChecker(const FCLCollisionChecker&); // Prevent construction by copying
//    FCLCollisionChecker& operator=(const FCLCollisionChecker&); // Prevent assignment
//    virtual ~FCLCollisionChecker(); // Prevent unwanted destruction
public:
    FCLCollisionChecker(){}
    FCLCollisionChecker(std::shared_ptr<RobotModel> &robot_model);
    virtual ~FCLCollisionChecker();

    void setRobotModel(std::shared_ptr<RobotModel> robot_model){ m_robot_model_ = robot_model; }

    // CollisionChecker interface
    void setDistanceTolerance(float distance);
    double getContactDistance();
    void getContinuousCollisionInfo(const DblVec &startjoints, const DblVec &endjoints, vector<Collision> &collisions);
    void getDiscreteCollisionInfo(vector<Collision> &collisions);
private:
    void transformPointToLocalFrame(const std::string &link_name, const Eigen::Vector3d &point_in, Eigen::Vector3d &point_out);
    void transformNormalToLocalFrame(const std::string &link_name, const Eigen::Vector3d &normal_in, Eigen::Vector3d &normal_out);
};

typedef boost::shared_ptr<FCLCollisionChecker> FCLCollisionCheckerPtr;

#endif // FCLCOLLISIONCHECKER_H
