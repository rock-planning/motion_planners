#include "RobotLink.hpp"
#include <boost/make_shared.hpp>

using namespace motion_planners;

RobotLink::RobotLink()
{}

std::vector<urdf::Pose> & RobotLink::getLinkCollisionRelativePose()
{
    return link_collision_relative_pose_;
}

std::vector<urdf::Pose> & RobotLink::getLinkVisualRelativePose()
{
    return link_visual_relative_pose_;
}

void RobotLink::calculateLinkVisualsPoseInGlobalPose( )
{
    KDL::Frame visual_global_pose;
    double x,y,z,w;

    for(std::size_t i = 0; i < link_visuals_.size(); i++)
    {
        visual_global_pose = getLinkFrame() * toKdl(link_visual_relative_pose_[i] );
        visual_global_pose.M.GetQuaternion(x,y,z,w);

        link_visuals_.at(i).origin.position.x = visual_global_pose.p.x();
        link_visuals_.at(i).origin.position.y = visual_global_pose.p.y();
        link_visuals_.at(i).origin.position.z = visual_global_pose.p.z();
        link_visuals_.at(i).origin.rotation.w = w;
        link_visuals_.at(i).origin.rotation.x = x;
        link_visuals_.at(i).origin.rotation.y = y;
        link_visuals_.at(i).origin.rotation.z = z;
    }
}

void RobotLink::calculateLinkCollisionPoseinGlobalPose( )
{
    KDL::Frame collision_global_pose;
    double x,y,z,w;
    for(std::size_t i = 0; i < link_collisions_.size(); i++)
    {
        collision_global_pose = getLinkFrame() * toKdl(link_collision_relative_pose_[i]);
        collision_global_pose.M.GetQuaternion(x,y,z,w);

        link_collisions_.at(i).origin.position.x = collision_global_pose.p.x();
        link_collisions_.at(i).origin.position.y = collision_global_pose.p.y();
        link_collisions_.at(i).origin.position.z = collision_global_pose.p.z();
        link_collisions_.at(i).origin.rotation.w = w;
        link_collisions_.at(i).origin.rotation.x = x;
        link_collisions_.at(i).origin.rotation.y = y;
        link_collisions_.at(i).origin.rotation.z = z;
    }
}

std::vector<urdf::VisualSharedPtr > RobotLink::getLinkVisuals()
{
    std::vector<urdf::VisualSharedPtr >  link_visuals_shared_ptr;
    for(std::size_t i = 0; i < link_visuals_.size(); i++ )
    {
        link_visuals_shared_ptr.push_back(urdf::VisualSharedPtr(new urdf::Visual(link_visuals_.at(i)))  );
    }
    return link_visuals_shared_ptr;
}

void RobotLink::setLinkVisuals( std::vector<urdf::VisualSharedPtr > &links)
{
    for(std::size_t i = 0; i < links.size(); i++)
    {
        link_visuals_.push_back(*(links.at(i).get())) ;
        link_visual_relative_pose_.push_back(links.at(i)->origin);
    }
}

void RobotLink::setLinkCollisions( std::vector<urdf::CollisionSharedPtr > &links)
{
    for(std::size_t i = 0; i < links.size(); i++)
    {
        link_collisions_.push_back( *(links.at(i).get()) );
        link_collision_relative_pose_.push_back(links.at(i)->origin);
    }
}

void RobotLink::setLinkCollision( const urdf::CollisionSharedPtr &link_collision)
{
    link_collisions_.push_back( *(link_collision.get()) );
    link_collision_relative_pose_.push_back(link_collision->origin);
}

void RobotLink::getLinkVisuals(std::vector<urdf::VisualSharedPtr > &link_visuals )
{
    for(std::size_t i = 0; i < link_visuals_.size();i++)
    {
        link_visuals.push_back( urdf::VisualSharedPtr(new urdf::Visual(link_visuals_.at(i))) );
    }

}

std::vector<urdf::CollisionSharedPtr > RobotLink::getLinkCollisions()
{
    //return this->link_collisions;
    std::vector<urdf::CollisionSharedPtr > link_collisions_share_ptr;

    for(std::size_t i = 0; i < link_collisions_.size();i++)
    {
        link_collisions_share_ptr.push_back( urdf::CollisionSharedPtr(new urdf::Collision(link_collisions_.at(i)))  );
    }
    return link_collisions_share_ptr;
}

void RobotLink::getLinkCollisions(std::vector<urdf::CollisionSharedPtr > &link_collision )
{
    //for(std::size_t i = 0; i < link_collisions_.size(); i++)
    for(auto &lc: link_collisions_)
    {
        //link_collision.push_back( urdf::CollisionSharedPtr(new urdf::Collision(link_collisions_.at(i))) );
        link_collision.push_back( urdf::CollisionSharedPtr(new urdf::Collision(lc)) );
    }
}

std::string &RobotLink::getLinkName()
{
    return link_name_;
}

void RobotLink::setLinkDFSVisited(bool visited)
{
    dfs_visited_  = visited;
}

bool RobotLink::getLinkDFSVisited()
{
    return dfs_visited_;
}

void RobotLink::setLinkName(std::string &link_name)
{
    link_name_ = link_name;
}

void RobotLink::setLinkFrame(KDL::Frame &link_frame)
{
    link_frame_ = link_frame;
}

KDL::Frame RobotLink::getLinkFrame()
{
    return link_frame_;
}

void RobotLink::AddCollision(urdf::CollisionSharedPtr collision)
{

    link_collision_relative_pose_.push_back(collision->origin);

    KDL::Frame collision_global_pose;
    double x,y,z,w;

    collision_global_pose = getLinkFrame() * toKdl(collision->origin);
    collision_global_pose.M.GetQuaternion(x,y,z,w);


    collision->origin.position.x = collision_global_pose.p.x();
    collision->origin.position.y = collision_global_pose.p.y();
    collision->origin.position.z = collision_global_pose.p.z();
    collision->origin.rotation.w = w;
    collision->origin.rotation.x = x;
    collision->origin.rotation.y = y;
    collision->origin.rotation.z = z;
    
    link_collisions_.push_back(*collision.get());
}


/* Subtracting the pointcloud using this method is not efficient because it creates a convex hull for the entire robot.
 * In doing so, computation time is less but it will delete all the points between the robot links.
 *
 * The commented functions are not used due to the reason mentioned above and also self-filter should be done outside the motion planner
 * Once in a while we also had issues with PCL and c++11. So due to the reasons mentioned above, functions related to plc based self filter are commented.
 */
/*
void RobotLink::setVisualPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > link_point_clouds)
{
    link_visual_point_clouds_ = link_point_clouds;
}

std::vector<pcl::PointCloud<pcl::PointXYZ> > RobotLink::getVisualPointCloud()
{
    return link_visual_point_clouds_;
}


void RobotLink::setCollisionPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > link_point_clouds)
{
    link_collision_point_clouds_ = link_point_clouds;
}

std::vector<pcl::PointCloud<pcl::PointXYZ> >& RobotLink::getCollisionPointCloud()
{
    return link_collision_point_clouds_;
}*/
