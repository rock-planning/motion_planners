#ifndef ROBOTLINK_HPP
#define ROBOTLINK_HPP

#include <vector>
#include <string>

#include <urdf_model/model.h>
#include <kdl/frames.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "HelperFunctions.hpp"

namespace motion_planners
{

class RobotLink
{

    public:
        RobotLink();
        std::vector<urdf::Pose> & getLinkCollisionRelativePose();
        std::vector<urdf::Pose> & getLinkVisualRelativePose();
        void calculateLinkVisualsPoseInGlobalPose();
        void calculateLinkCollisionPoseinGlobalPose();
        std::vector<urdf::VisualSharedPtr >  getLinkVisuals();
        void setLinkVisuals( std::vector<urdf::VisualSharedPtr > &link_visuals);
        void setLinkCollisions( std::vector<urdf::CollisionSharedPtr > &link_collisions);
        void setLinkCollision( const urdf::CollisionSharedPtr &link_collision);
        void getLinkVisuals(std::vector<urdf::VisualSharedPtr > &link_visuals );
        void getLinkCollisions(std::vector<urdf::CollisionSharedPtr > &link_collision);
        void setVisualPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > link_point_clouds);
        std::vector<pcl::PointCloud<pcl::PointXYZ> > getVisualPointCloud();

        std::vector<urdf::CollisionSharedPtr >  getLinkCollisions();

        std::string &getLinkName();
        void setLinkDFSVisited(bool visited);
        bool getLinkDFSVisited();
        void setLinkName(std::string &link_name);
        void setLinkFrame(KDL::Frame &link_frame);
        KDL::Frame getLinkFrame();
        void AddCollision(urdf::CollisionSharedPtr collision);
        void setCollisionPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > link_point_clouds);
        std::vector<pcl::PointCloud<pcl::PointXYZ> >& getCollisionPointCloud();

    private:

        bool dfs_visited_;
        std::string link_name_;
        KDL::Frame link_frame_;
        std::vector<urdf::Visual > link_visuals_;
        std::vector<urdf::Collision> link_collisions_;
        std::vector<pcl::PointCloud<pcl::PointXYZ> > link_visual_point_clouds_, link_collision_point_clouds_;
        std::vector<urdf::Pose> link_visual_relative_pose_, link_collision_relative_pose_;

};
}// end namespace motion_planners
#endif // ROBOTLINK_HPP
















