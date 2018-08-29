#ifndef FCLCOLLISIONCHECKER_H
#define FCLCOLLISIONCHECKER_H

#include "trajopt/collision_checker.hpp"
#include "RobotModel.hpp"

using namespace trajopt;
using namespace motion_planners;

class FCLCollisionChecker : public CollisionChecker
{

    double m_contact_tolerance;
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
    void SetContactDistance(float distance);
    double GetContactDistance();
    void GetContinuousCollisionInfo(const DblVec &startjoints, const DblVec &endjoints, vector<Collision> &collisions);
    void GetDiscreteCollisionInfo(vector<Collision> &collisions);
};

typedef boost::shared_ptr<FCLCollisionChecker> FCLCollisionCheckerPtr;

#endif // FCLCOLLISIONCHECKER_H
