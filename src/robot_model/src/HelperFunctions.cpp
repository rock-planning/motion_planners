
#include "HelperFunctions.hpp"


namespace motion_planners
{

KDL::Vector toKdl(urdf::Vector3 v)
{
    return KDL::Vector(v.x, v.y, v.z);
}

// construct rotation
KDL::Rotation toKdl(urdf::Rotation r)
{
    return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
KDL::Frame toKdl(urdf::Pose p)
{
    return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
}

urdf::Pose toURDFPose(KDL::Frame frame)
{
    urdf::Pose urdf_pose;
    urdf_pose.position.x=frame.p.x();
    urdf_pose.position.y=frame.p.y();
    urdf_pose.position.z=frame.p.z();

    double x,y,z,w;
    frame.M.GetQuaternion(x,y,z,w);

    urdf_pose.rotation.x=x;
    urdf_pose.rotation.y=y;
    urdf_pose.rotation.z=z;
    urdf_pose.rotation.w=w;
    return urdf_pose;
}

void KDLFrameToEigenMatrix(KDL::Frame &frame,Eigen::Affine3f &transform)
{
    transform.translation() <<frame.p.x() ,frame.p.y() ,frame.p.z();
    transform(0,0)=frame.M(0,0);
    transform(0,1)=frame.M(0,1);
    transform(0,2)=frame.M(0,2);
    transform(1,0)=frame.M(1,0);
    transform(1,1)=frame.M(1,1);
    transform(1,2)=frame.M(1,2);
    transform(2,0)=frame.M(2,0);
    transform(2,1)=frame.M(2,1);
    transform(2,2)=frame.M(2,2);
    transform(3,0)=0;
    transform(3,1)=0;
    transform(3,1)=0;
    transform(3,3)=1;
}

}


