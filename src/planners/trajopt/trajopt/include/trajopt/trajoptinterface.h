#ifndef TRAJOPTINTERFACE_H
#define TRAJOPTINTERFACE_H

#pragma once
#include "typedefs.hpp"
#include "utils/macros.h"
#include "trajopt/transform.h"

namespace trajopt {

class  CollisionChecker;
class Collision;

typedef int CLA_CheckLimits;

class Link;
typedef boost::shared_ptr<Link> LinkPtr;

class Link{
public:
    //    std::string GetName() const{return "Not Implemented";}
    //    inline geometry::Transform GetTransform() const {

    //    }
    //    /// \brief unique index into parent KinBody::GetLinks vector
    //    inline int GetIndex() const {
    //        return _index;
    //    }

    virtual std::string GetName() const = 0;
    virtual geometry::Transform GetTransform()  = 0;

    /// \brief unique index into parent KinBody::GetLinks vector
    virtual int GetIndex() const  = 0;
    virtual LinkPtr getParent() = 0;
};

//enum geo_type{
//    Box,
//    Cylinder,
//    sphere,
//    TriMesh
//};

//class Geometry{
//    int _type;
//public:
//    int GetType(){
//        return _type;
//    }

//};
//typedef boost::shared_ptr<Geometry> GeometryPtr;


class KinBody{

public:
    vector<LinkPtr> GetLinks() const{
        vector<LinkPtr> out;

        return out;
    }
    std::string GetName() const{return "Not Implemented";}

    int GetDOF() const { return 0;}

    //    void SetDOFValues(const std::vector<dReal>& values, uint32_t checklimits = CLA_CheckLimits, const std::vector<int>& dofindices = std::vector<int>()){
    void SetDOFValues(const std::vector<double>& values){
    }

    void SetTransform(const geometry::Transform& transform){}



};
typedef boost::shared_ptr<KinBody> KinBodyPtr;

class TRAJOPT_API Configuration {
public:

    virtual void SetDOFValues(const DblVec& dofs) = 0;
    virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const = 0;
    virtual DblVec GetDOFValues() = 0;
    virtual int GetDOF() const = 0;
    virtual DblMatrix PositionJacobian(std::string link_name, const Vector3d& pt) const = 0;
    virtual DblMatrix RotationJacobian(std::string link_name) const = 0;
    //  virtual bool DoesAffect(const T& link) = 0;
    //  virtual vector<T> GetBodies() = 0;
    //  virtual std::vector<T> GetAffectedLinks() = 0;
    virtual void GetAffectedLinks(std::vector<LinkPtr>& links, bool only_with_geom, vector<std::string>& link_inds) = 0;
    virtual DblVec RandomDOFValues() = 0;

    virtual geometry::Transform GetLinkTransformByName(std::string link_name) = 0;
//    virtual boost::shared_ptr<CollisionChecker> getCollisionChecker() = 0;

    virtual ~Configuration() {}
    struct Saver {
        virtual ~Saver(){}
    };
    typedef boost::shared_ptr<Saver> SaverPtr;
    struct GenericSaver : public Saver {
        DblVec dofvals;
        Configuration* parent;
        GenericSaver(Configuration* _parent) : dofvals(_parent->GetDOFValues()), parent(_parent) {}
        ~GenericSaver() {
            parent->SetDOFValues(dofvals);
        }
    }; // inefficient

    virtual SaverPtr Save() {
        return SaverPtr(new GenericSaver(this));
    }


};
typedef boost::shared_ptr<Configuration> ConfigurationPtr;

/**
Each CollisionChecker object has a copy of the world, so for performance, don't make too many copies
*/
class TRAJOPT_API CollisionChecker {
public:

    /** contacts of distance < (arg) will be returned */
    virtual void SetContactDistance(float distance)  = 0;
    virtual double GetContactDistance() = 0;

    //  virtual void ContinuousCheckTrajectory(const TrajArray& traj, Configuration& rad, vector<Collision>& collisions) {throw std::runtime_error("not implemented");}

    /** Find contacts between swept-out shapes of robot links and everything in the environment, as robot goes from startjoints to endjoints */
    virtual void GetContinuousCollisionInfo(const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions) = 0;

    virtual void GetDiscreteCollisionInfo(vector<Collision> &collisions) = 0;

    //  /** Finds all self collisions when all joints are set to zero, and ignore collisions between the colliding links */
    //  void IgnoreZeroStateSelfCollisions();
    //  void IgnoreZeroStateSelfCollisions(KinBodyPtr body);

    //  /** Prevent this pair of links from colliding */
    //  virtual void ExcludeCollisionPair(const Link& link0, const Link& link1) = 0;
    //  virtual void IncludeCollisionPair(const Link& link0, const Link& link1) = 0;

    //  /** Check whether a raycast hits the environment */
    //  virtual bool RayCastCollision(const Vector3d& point1, const Vector3d& point2) = 0;


    //  OpenRAVE::EnvironmentBaseConstPtr GetEnv() {return m_env;}

    virtual ~CollisionChecker() {}
    /** Get or create collision checker for this environment */
//    static boost::shared_ptr<CollisionChecker> GetOrCreate();
protected:
    CollisionChecker(){}
};
typedef boost::shared_ptr<CollisionChecker> CollisionCheckerPtr;

CollisionCheckerPtr TRAJOPT_API CreateCollisionChecker();

}

#endif // TRAJOPTINTERFACE_H
