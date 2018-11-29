#ifndef HELPERFUNCTIONS_HPP
#define HELPERFUNCTIONS_HPP


#include <vector>

#include <kdl/frames.hpp>
#include <urdf_parser/urdf_parser.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace motion_planners
{

    KDL::Vector toKdl(urdf::Vector3 v);
    KDL::Rotation toKdl(urdf::Rotation r);
    KDL::Frame toKdl(urdf::Pose p);
    urdf::Pose toURDFPose(KDL::Frame frame);
    void KDLFrameToEigenMatrix(KDL::Frame &frame,Eigen::Isometry3f &transform);
}

#endif

