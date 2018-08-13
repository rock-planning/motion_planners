#include "RobotModel.hpp"
#include <omp.h>
#include <iostream>
#include <boost/iterator/iterator_concepts.hpp>

namespace motion_planners
{

/////////////////////////////////////////////////out of class  function for comparing link name//////////////////////////////
std::string _removeLinkName;

bool isLinkListed(const urdf::LinkSharedPtr &remove_link)
{
    return (remove_link->name == _removeLinkName);
}

/////////////////////////////////////////////////End of out of class members and variables////////////////////////////////////

RobotModel::RobotModel(std::string urdf_file, std::string srdf_file, std::string planning_group_name,
                       double link_padding  )
{
    urdf_file_abs_path_  = urdf_file;
    srdf_file_abs_path_  = srdf_file;
    planning_group_name_ = planning_group_name;
    link_padding_        = link_padding;
    
    //initialization();
}


/*
RobotModel::RobotModel(std::string urdf_file_abs_path,
                       std::string srdf_file_abs_path,
		               std::vector<manipulator_planner_library::JointWeight> jt_weight,
                       std::string ik_fast_shared_object_abs_path,
		               double link_padding)
{

    RobotModel(urdf_file_abs_path, srdf_file_abs_path, link_padding);
    
    this->ik_fast_shared_object_abs_path = ik_fast_shared_object_abs_path ;

    if (!mJointsWeight.empty())
      mJointsWeight = jt_weight;

}*/

void RobotModel::setWorldCollisionDetector(collision_detection::AbstractCollisionPtr collision_detector)
{
    world_collision_detector_ = collision_detector;    
    robot_collision_detector_->assignWorldDetector(world_collision_detector_);
    
}

bool RobotModel::initialiseURDFandSRDF()
{
	std::string xml_string;
	std::fstream xml_file(urdf_file_abs_path_.c_str(), std::fstream::in);
	bool srdf_ok_ = false ;

	srdf_model_.reset(new srdf::Model());
	if (xml_file.is_open())
	{
	    while ( xml_file.good() )
	    {
	        std::string line;
	        std::getline( xml_file, line);
	        xml_string += (line + "\n");
	    }
	    xml_file.close();
	    urdf_model_ = urdf::parseURDF(xml_string);
	    if(urdf_model_.get() == NULL)
		{
	    	LOG_ERROR("[RobotModel] Error while getting urdf model. urdf_model is empty");
			return false;
		}

	}
	else
	{
	    LOG_ERROR("[RobotModel] Cannot open urdf file.");
		return false;
	}

	srdf_ok_ = srdf_model_->initFile(*urdf_model_, srdf_file_abs_path_);

	if(!srdf_ok_)
	{
	    LOG_ERROR("[RobotModel] Error while initialising srdf model");
		return srdf_ok_;
	}

    if (!kdl_parser::treeFromFile(urdf_file_abs_path_, kdl_tree_))
	{
	    LOG_ERROR("[RobotModel] Error while initialising kdl treey");
	    return false;
	}

	return true;
}

bool RobotModel::initialization()
{
	// initialse urdf and srdf
	bool res = initialiseURDFandSRDF();

	if(!res)
	{
		LOG_FATAL("[RobotModel] Robot model initialisation failed");
		return false;
	}

    // Initialise the robot joints
    double joint_value;
    std::string joint_name;
    RobotJoint robot_joint;

    for( std::map<std::string, urdf::JointSharedPtr >::iterator it = urdf_model_->joints_.begin(); it != urdf_model_->joints_.end(); it++)
    {
        LOG_DEBUG("[RobotModel] Visiting the joint:%s and it is of type %d", it->first.c_str(), it->second->type );

        if( (it->second->type != urdf::Joint::FIXED) && (it->second->type != urdf::Joint::UNKNOWN) && (it->second->type != urdf::Joint::CONTINUOUS) )
        {
            if( (it->second->limits->lower > 0) || (it->second->limits->upper < 0) )
            {
                joint_value = (it->second->limits->lower + it->second->limits->upper )/2.0;
            }
            else
            {
                joint_value = 0;
            }
            joint_name = it->first;
            robot_joint.setJointValue(joint_value);
            robot_joint.setJointName(joint_name);
            robot_joint.setJointInfo(*(it->second.get()));
            // incase of mimic joints
            if (it->second->mimic)
            {		
                robot_joint.setJointAsMimic();
                robot_joint.mimic_joints_ = MimicJoint( it->second->mimic->joint_name,
                                                        it->second->mimic->multiplier, it->second->mimic->offset );

            }
            // Assign robot joint to the robot state
	    robot_state_.robot_joints_[joint_name] = robot_joint;
        }
    }

    for (std::map<std::string, RobotJoint>::iterator it = robot_state_.robot_joints_.begin(); it!= robot_state_.robot_joints_.end(); ++it)
    {
        if(it->second.isMimicJoint())
        {
            if( robot_state_.robot_joints_.find( it->second.mimic_joints_.joint_to_mimic ) == robot_state_.robot_joints_.end() )
            {
                LOG_DEBUG("[RobotModel]: The mimic joint name: %s is not available", it->second.mimic_joints_.joint_to_mimic.c_str());
                return false;
            }
            else
            {
                robot_state_.robot_joints_[it->second.mimic_joints_.joint_to_mimic].mimic_joints_map_[it->first]  = robot_state_.robot_joints_[it->first].mimic_joints_;
            }
        }
    }

    // Initialise robot link
    std::string link_name;    
    RobotLink robot_link;    

    auto start_initialisation = std::chrono::high_resolution_clock::now();
    

    for(std::map<std::string, urdf::LinkSharedPtr >::iterator it = urdf_model_->links_.begin(); it != urdf_model_->links_.end(); it++)
    {
        link_name = it->second->name;
        robot_link.setLinkDFSVisited(false);
        robot_link.setLinkName(link_name);
        robot_state_.robot_links_[link_name] = robot_link;

        //get visual and collision data from urdf and assign to the robot state
        std::vector<urdf::VisualSharedPtr > visual_array        = urdf_model_->getLink(link_name)->visual_array;
        std::vector<urdf::CollisionSharedPtr > collision_array  = urdf_model_->getLink(link_name)->collision_array;
        robot_state_.robot_links_[link_name].setLinkVisuals( visual_array );
        robot_state_.robot_links_[link_name].setLinkCollisions(collision_array );

        //convert visual and collision data to pointcloud and then assing to the robot state
        //TODO: Why are we doing this ?
        std::vector<pcl::PointCloud<pcl::PointXYZ> > link_visual_point_cloud, link_collision_point_cloud;
        createPointCloudFromVisual(visual_array, link_visual_point_cloud);
        createPointCloudFromCollision(collision_array, link_collision_point_cloud );
        robot_state_.robot_links_[link_name].setVisualPointCloud(link_visual_point_cloud);
        robot_state_.robot_links_[link_name].setCollisionPointCloud(link_collision_point_cloud);

    }

    auto finish_initialisation = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<double> elapsed = finish_initialisation - start_initialisation;
    
    std::cout<<"[RobotModel] Robot link initialisation took "<< elapsed.count()<<" seconds"<<std::endl;
    LOG_INFO("[RobotModel] Robot link initialisation took %d seconds", elapsed.count());

    std::vector<std::string>  list_of_visited_link_from_root;
    dfsTraversing(urdf_model_->getRoot()->name, list_of_visited_link_from_root);
    settingVisitedFlagLinkToFalse();
    initializeLinksCollisions();

	return true;
}

void RobotModel::dfsTraversing(std::string start_link_name, std::vector<std::string>& visited_links)
{
    robot_state_.robot_links_[start_link_name].setLinkDFSVisited(true);
    visited_links.push_back(start_link_name);

    std::string child_link_name;
    int joint_type;
    std::string joint_name;
    bool is_link_visited;
    KDL::JntArray kdl_chain_joint_array;
    KDL::Frame Child_link_Frame_in_Parent_Frame;
    KDL::Frame Parent_link_in_base_link;
    KDL::Frame Child_link_Frame_in_base_link;

    std::vector<urdf::LinkSharedPtr > child_links = urdf_model_->getLink(start_link_name)->child_links;

    for(std::size_t i = 0; i < child_links.size(); i++)
    {
        child_link_name     = child_links.at(i)->name;
        is_link_visited     = robot_state_.robot_links_[child_link_name].getLinkDFSVisited();
        urdf::JointSharedPtr joint_between_child_link_and_parent_link = urdf_model_->getLink(child_link_name)->parent_joint;
        joint_name          = joint_between_child_link_and_parent_link->name;
        joint_type          = joint_between_child_link_and_parent_link->type;

        if(!is_link_visited)
        {
            kdl_tree_.getChain(start_link_name, child_link_name, kdl_chain_);
            KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
            kdl_chain_joint_array.resize(kdl_chain_.getNrOfJoints());
            if(joint_type != urdf::Joint::FIXED)
            {
                kdl_chain_joint_array.data[0]=robot_state_.robot_joints_[joint_name].getJointValue();
//                 LOG_DEBUG( "[RobotModel] DFSTraversing : The joint between %s and %s is of type %d and is of value %d", 
//                             start_link_name.c_str(), child_link_name.c_str(), joint_type, kdl_chain_joint_array.data[0] ); 
            }

            fk_solver.JntToCart(kdl_chain_joint_array, Child_link_Frame_in_Parent_Frame);

            // if the parent link is root no more composing frames, the link pose has been already calculated 
            if(start_link_name == urdf_model_->getRoot()->name)
            {
                robot_state_.robot_links_[child_link_name].setLinkFrame(Child_link_Frame_in_Parent_Frame);
                    
//                 LOG_DEBUG("============================================================================== ");
//                 LOG_DEBUG("Frame: %s", child_link_name.c_str());
//                 LOG_DEBUG("x ", Child_link_Frame_in_Parent_Frame.p.x());
//                 LOG_DEBUG("y ", Child_link_Frame_in_Parent_Frame.p.y());
//                 LOG_DEBUG("z ", Child_link_Frame_in_Parent_Frame.p.z());
//                 LOG_DEBUG("============================================================================== ");
            }
            else
            {
                Parent_link_in_base_link        = robot_state_.robot_links_[start_link_name].getLinkFrame();
                Child_link_Frame_in_base_link   = Parent_link_in_base_link * Child_link_Frame_in_Parent_Frame;
                robot_state_.robot_links_[child_link_name].setLinkFrame(Child_link_Frame_in_base_link);
              
//                 LOG_DEBUG("============================================================================== ");
//                 LOG_DEBUG("Frame: %s", child_link_name.c_str());
//                 LOG_DEBUG("x ", Child_link_Frame_in_base_link.p.x());
//                 LOG_DEBUG("y ", Child_link_Frame_in_base_link.p.y());
//                 LOG_DEBUG("z ", Child_link_Frame_in_base_link.p.z());
//                 LOG_DEBUG("============================================================================== ");
               
            }
            // now we have to calculate the position of the visuals and collision of the robot in the world coordinate (global pose)
            robot_state_.robot_links_[child_link_name].calculateLinkVisualsPoseInGlobalPose();
            robot_state_.robot_links_[child_link_name].calculateLinkCollisionPoseinGlobalPose();

            // recursive dfs 
            dfsTraversing(child_link_name, visited_links);

        }
    }
}

void RobotModel::settingVisitedFlagLinkToFalse()
{
    for( std::map<std::string, RobotLink>::iterator it = robot_state_.robot_links_.begin(); it != robot_state_.robot_links_.end(); it++ )
    {
        // std::cout<<"setting visited to false for the link: " << it->second.getLinkName()<<std::endl;
        it->second.setLinkDFSVisited(false);
    }
}

void RobotModel::settingVisitedFlagLinkToFalse(std::vector<std::string> visted_links_names)
{
    std::string link_name;
    for(std::size_t i=0; i < visted_links_names.size(); i++)
    {
        link_name = visted_links_names.at(i);
        //std::cout<<"setting visited to false for the link: " << link_name<<std::endl;
        robot_state_.robot_links_[link_name].setLinkDFSVisited(false);
    }
}

RobotState RobotModel::getRobotState()
{
    return robot_state_;
}

void RobotModel::setRobotState(RobotState &robot_state )
{
    robot_state_ = robot_state;
}
/*
void RobotModel::setDefaultJointWeight( const std::vector< std::string > &planning_joints_name)
{
    mJointsWeight_.resize(planning_joints_name.size());

    for(std::size_t i = 0; i < planning_joints_name.size(); i++)
    {
        mJointsWeight_.at(i).joint_name   = planning_joints_name.at(i);
        mJointsWeight_.at(i).weight       = 1.0;
    }
}

bool RobotModel::setJointWeight(const std::vector<manipulator_planner_library::JointWeight> &joints_weight, 
                                const std::vector< std::string > &planning_joints_name)
{

    mJointsWeight_.resize(joints_weight.size());

    for(std::size_t i = 0; i < planning_joints_name.size(); i++)
            LOG_DEBUG("[RobotModel] Setting weight:%d for the joint: %s", joints_weight.at(i).weight, planning_joints_name.at(i)); 

    std::vector< std::string>::const_iterator it;
    for(std::size_t i = 0; i < joints_weight.size(); i++)
    {
        it = std::find(planning_joints_name.begin(), planning_joints_name.end(), joints_weight.at(i).joint_name);

        if (it == planning_joints_name.end())
        {
            LOG_ERROR("[RobotModel] Setting joints: The joint name %s is not found in planning joint name", *it);
            return false;
        }
        
        mJointsWeight_.at(i).joint_name   = joints_weight.at(i).joint_name;
        mJointsWeight_.at(i).weight       = joints_weight.at(i).weight;
    }
    return true;
}
*/
void RobotModel::initializeLinksCollisions()
{
//#define INITIALIZELINKSCOLLISIONS_LOG
     
    boost::filesystem::path abs_path_of_mesh_file;
    std::string urdf_directory_path;

    // get the disabled collision pairs from srdf
    std::vector<srdf::Model::DisabledCollision> DisabledCollisionPairs = srdf_model_->getDisabledCollisionPairs();
    // assign the disabled collision pairs to the collision library
    robot_collision_detector_->AbstractCollisionDetection::setDisabledCollisionPairs( DisabledCollisionPairs);

    int total_number_of_collision_should_be = 0;    
    std::string link_name,  abs_path_to_mesh_file,  collision_object_name;
    double radius, length ,box_x, box_y, box_z;
    Eigen::Vector3d mesh_scale;
    std::vector<urdf::CollisionSharedPtr > link_collisions;

    for(std::map<std::string, RobotLink>::iterator it = robot_state_.robot_links_.begin(); it != robot_state_.robot_links_.end(); it++)
    {
        link_name 	= it->first;
        link_collisions = it->second.getLinkCollisions();
        
        LOG_DEBUG("For the link:%s there are %i collision objects in this link", link_name.c_str(), link_collisions.size());
        total_number_of_collision_should_be = total_number_of_collision_should_be + link_collisions.size();
        

        for(std::size_t i = 0; i < link_collisions.size(); i++ )
        {            
            collision_object_name = link_name+"_" +lexical_cast<std::string>(i);

	    base::Pose collision_object_pose;
	    collision_object_pose.position.x() = link_collisions.at(i)->origin.position.x;
	    collision_object_pose.position.y() = link_collisions.at(i)->origin.position.y;
	    collision_object_pose.position.z() = link_collisions.at(i)->origin.position.z;
	    
            collision_object_pose.orientation.w() = link_collisions.at(i)->origin.rotation.w;
	    collision_object_pose.orientation.x() = link_collisions.at(i)->origin.rotation.x;
	    collision_object_pose.orientation.y() = link_collisions.at(i)->origin.rotation.y;
	    collision_object_pose.orientation.z() = link_collisions.at(i)->origin.rotation.z;
                        
	    LOG_DEBUG(" The collision object name is %s", collision_object_name.c_str() );
            LOG_DEBUG_S<<"x: "<< link_collisions.at(i)->origin.position.x;
            LOG_DEBUG_S<<"y: "<< link_collisions.at(i)->origin.position.y;
            LOG_DEBUG_S<<"z: "<< link_collisions.at(i)->origin.position.z;
	    

            if(link_collisions.at(i)->geometry->type == urdf::Geometry::MESH)
            {
                LOG_DEBUG("------------------------------registering mesh file------------------------------ " );
                
                urdf::MeshSharedPtr urdf_mesh_ptr = urdf::static_pointer_cast <urdf::Mesh> (link_collisions.at(i)->geometry);
		
                mesh_scale(0) = urdf_mesh_ptr->scale.x;
                mesh_scale(1) = urdf_mesh_ptr->scale.y;
                mesh_scale(2) = urdf_mesh_ptr->scale.z;
		                                
                urdf_directory_path = urdf_file_abs_path_.substr(0, urdf_file_abs_path_.find_last_of("/") );

                abs_path_of_mesh_file = resolve_path( urdf_mesh_ptr->filename, urdf_directory_path );

                abs_path_to_mesh_file=abs_path_of_mesh_file.string();

                robot_collision_detector_->registerMeshToCollisionManager(abs_path_to_mesh_file, mesh_scale, collision_object_name, collision_object_pose, link_padding_);
            }
            else if(link_collisions.at(i)->geometry->type == urdf::Geometry::BOX)
            {

                #ifdef INITIALIZELINKSCOLLISIONS_LOG
                    std::cout<<"------------------------------registering box------------------------------ " <<std::endl;
                #endif

                urdf::BoxSharedPtr urdf_box_ptr = urdf::static_pointer_cast <urdf::Box> (link_collisions.at(i)->geometry);
                box_x = urdf_box_ptr->dim.x;
                box_y = urdf_box_ptr->dim.y;
                box_z = urdf_box_ptr->dim.z;
                robot_collision_detector_->registerBoxToCollisionManager( box_x, box_y, box_z,collision_object_name, collision_object_pose, link_padding_);
            }
            else if(link_collisions.at(i)->geometry->type == urdf::Geometry::CYLINDER)
            {

                #ifdef INITIALIZELINKSCOLLISIONS_LOG
                    std::cout<<"------------------------------registering cylinder------------------------------" <<std::endl;
                #endif

                urdf::CylinderSharedPtr urdf_cylinder_ptr= urdf::static_pointer_cast <urdf::Cylinder> (link_collisions.at(i)->geometry);
                radius=urdf_cylinder_ptr->radius;
                length=urdf_cylinder_ptr->length;
                robot_collision_detector_->registerCylinderToCollisionManager(radius , length, collision_object_name, collision_object_pose, link_padding_);
            }
            else if(link_collisions.at(i)->geometry->type == urdf::Geometry::SPHERE)
            {

                #ifdef INITIALIZELINKSCOLLISIONS_LOG
                    std::cout<<"------------------------------registering sphere------------------------------ " <<std::endl;
                #endif

                urdf::SphereSharedPtr urdf_sphere_ptr= urdf::static_pointer_cast <urdf::Sphere> (link_collisions.at(i)->geometry);
                radius=urdf_sphere_ptr->radius;
                robot_collision_detector_->registerSphereToCollisionManager(radius,collision_object_name, collision_object_pose, link_padding_);
            }
        }
        #ifdef INITIALIZELINKSCOLLISIONS_LOG
        std::cout<<"======================================== finish visiting "<< link_name<<"=======================================" <<std::endl;
        #endif
    }

    #ifdef INITIALIZELINKSCOLLISIONS_LOG
    std::cout<<"===========================================end initializeLinksCollisions===========================================" <<std::endl;
    #endif
    
    std::cout<<"NUmber disabpled collision pari = "<<robot_collision_detector_->AbstractCollisionDetection::disabled_collisions_.size()<<std::endl;

    return;
}



void RobotModel::scalePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double scale_x, double scale_y, double scale_z)
{
    double cloud_in_x,cloud_in_y,cloud_in_z;

    for(std::size_t i=0; i < cloud_in->size();i++)
    {
        cloud_in_x=cloud_in->at(i).x;
        cloud_in_y=cloud_in->at(i).y;
        cloud_in_z=cloud_in->at(i).z;
        cloud_out->push_back(pcl::PointXYZ(cloud_in_x /scale_x,cloud_in_y /scale_y, cloud_in_z /scale_z) );
    }
}

void RobotModel::createPtCloudFromBox(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_box_cloud_ptr,
                                            double x, double y, double z, Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix, bool dense)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>()) ;
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,y/2 ,-z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,-y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,-y/2 ,-z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,y/2 ,-z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,-y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,-y/2 ,-z/2 ) );

    if(dense)
    {
        double delta_x= 0.02;
        double delta_y= 0.02;
        double delta_z= 0.02;

        for(double deltaX = -x/2 + delta_x; deltaX < x/2; deltaX = deltaX+delta_x)
        {
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,-z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,-z/2 ) );

            for(double deltaZ = -z/2 + delta_z; deltaZ < z/2; deltaZ = deltaZ+delta_z)
            {
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,deltaZ ) );

            }

            for(double deltaY = -y/2 + delta_y; deltaY < y/2; deltaY = deltaY+delta_y)
            {
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,z/2 ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,-z/2 ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,z/2 ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,-z/2 ) );}

        }

        for(double deltaY = -y/2 + delta_y; deltaY < y/2; deltaY = deltaY+delta_y)
        {
            box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,-z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,-z/2 ) );

            for(double deltaZ = -z/2 + delta_z; deltaZ < z/2; deltaZ = deltaZ+delta_z)
            {
                box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,deltaZ ) );

            }

        }

        /*for(double deltaZ = -z/2 + delta_z; deltaZ < z/2; deltaZ = deltaZ+delta_z)
        {
            box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,y/2 ,deltaZ ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,-y/2 ,deltaZ ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,-y/2 ,deltaZ ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,y/2 ,deltaZ ) );

        }*/

    }

    pcl::transformPointCloud (*box_cloud_ptr, *transformed_box_cloud_ptr, link_visual_pose_in_sensor_frame_eigen_matrix);
}

void RobotModel::createPtCloudFromCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cylinder_cloud_ptr,
                                                 double radius, double height, Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix,
                                                 int number_of_step_alpha, bool dense )
{
    double alpha_angle=M_PI/number_of_step_alpha;
    double z=height/2;
    double x,y;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
    {
        x=radius*cos(alpha);
        y=radius*sin(alpha);
        cylinder_cloud_ptr->push_back(pcl::PointXYZ(x,y,z));
        cylinder_cloud_ptr->push_back(pcl::PointXYZ(x,y,-z));
    }

    if (dense)
    {
        double delta_z = z * 0.1;
        for(double delta = delta_z; delta < z; delta = delta+delta_z)
        {
            for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
            {
                x=radius*cos(alpha);
                y=radius*sin(alpha);
                cylinder_cloud_ptr->push_back(pcl::PointXYZ(x,y,delta));
                cylinder_cloud_ptr->push_back(pcl::PointXYZ(x,y,-delta));
            }
        }
        for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
        {
            x=radius*cos(alpha);
            y=radius*sin(alpha);
            cylinder_cloud_ptr->push_back(pcl::PointXYZ(x,y,0.0));
        }
    }

    pcl::transformPointCloud (*cylinder_cloud_ptr, *transformed_cylinder_cloud_ptr, link_visual_pose_in_sensor_frame_eigen_matrix);
}

void RobotModel::createPtCloudFromSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_sphere_cloud_ptr, double radius, 
					 Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix, int number_of_step_alpha, int number_of_step_beta)
{

    // alpha cretae point on a circle given radius, beta will give you the radius,

    double alpha_angle, beta_angle;
    double x_origin, y_origin, z_origin;

    x_origin=link_visual_pose_in_sensor_frame_eigen_matrix(0,3);
    y_origin=link_visual_pose_in_sensor_frame_eigen_matrix(1,3);
    z_origin=link_visual_pose_in_sensor_frame_eigen_matrix(2,3);


    alpha_angle= M_PI/number_of_step_alpha;
    beta_angle= M_PI/number_of_step_beta;


    double x,y,z,new_radius;

    for(double beta=0;beta<M_PI/2;beta=beta+beta_angle)
    {
        new_radius=radius*cos(beta);
        z=radius*sin(beta);

        for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
        {
            x=new_radius*cos(alpha)+x_origin;
            y=new_radius*sin(alpha)+y_origin;
            transformed_sphere_cloud_ptr->push_back(pcl::PointXYZ(x,y,z+z_origin));
            transformed_sphere_cloud_ptr->push_back(pcl::PointXYZ(x,y,-z+z_origin));
        }
    }

    //top and bottom of the sphere
    transformed_sphere_cloud_ptr->push_back(pcl::PointXYZ(x_origin,y_origin,z_origin+radius));
    transformed_sphere_cloud_ptr->push_back(pcl::PointXYZ(x_origin,y_origin,z_origin-radius));


}

void RobotModel::createPtCloudFromBox(pcl::PointCloud<pcl::PointXYZ> &box_cloud,double x, double y, double z)
{
    box_cloud.push_back(pcl::PointXYZ(x/2 ,y/2 ,z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(x/2 ,y/2 ,-z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(x/2 ,-y/2 ,z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(x/2 ,-y/2 ,-z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(-x/2 ,y/2 ,z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(-x/2 ,y/2 ,-z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(-x/2 ,-y/2 ,z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(-x/2 ,-y/2 ,-z/2 ) );
}

void RobotModel::createPtCloudFromCylinder(pcl::PointCloud<pcl::PointXYZ> &cylinder_cloud, double radius, double height, int number_of_step_alpha )
{
    double alpha_angle=M_PI/number_of_step_alpha;
    double z=height/2;
    double x,y;
    for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
    {
        x=radius*cos(alpha);
        y=radius*sin(alpha);
        cylinder_cloud.push_back(pcl::PointXYZ(x,y,z));
        cylinder_cloud.push_back(pcl::PointXYZ(x,y,-z));
    }
}

void RobotModel::createPtCloudFromSphere(pcl::PointCloud<pcl::PointXYZ> &sphere_cloud, double radius, int number_of_step_alpha, int number_of_step_beta)
{
    // alpha cretae point on a circle given radius, beta will give you the radius,
    double alpha_angle, beta_angle;
    alpha_angle= M_PI/number_of_step_alpha;
    beta_angle= M_PI/number_of_step_beta;
    double x,y,z,new_radius;
    for(double beta=0;beta<M_PI/2;beta=beta+beta_angle)
    {
        new_radius=radius*cos(beta);
        z=radius*sin(beta);
        for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
        {
            x=new_radius*cos(alpha);
            y=new_radius*sin(alpha);
            sphere_cloud.push_back(pcl::PointXYZ(x,y,z));
            sphere_cloud.push_back(pcl::PointXYZ(x,y,-z));
        }
    }
    //top and bottom of the sphere
    sphere_cloud.push_back(pcl::PointXYZ(0,0,+radius));
    sphere_cloud.push_back(pcl::PointXYZ(0,0,-radius));
}

void RobotModel::subtractingPtClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene_ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr  scene_ptr ,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr )
{
    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> hull;
// Set alpha, which is the maximum length from a vertex to the center of the voronoi cell (the smaller, the greater the resolution of the hull).
//    http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_3:_Cloud_processing_(advanced)#Concave_hull
/*
    pcl::ConcaveHull<pcl::PointXYZ> hull;
    hull.setAlpha(0.1);
*/
    hull.setInputCloud(cloud_ptr);
    hull.setDimension(3);
    hull.reconstruct(*boundingbox_ptr.get(),polygons);


    std::vector<int> indices;
    pcl::CropHull<pcl::PointXYZ> bb_filter;

    bb_filter.setDim(3);
    bb_filter.setInputCloud(scene_ptr);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(boundingbox_ptr);
    bb_filter.filter(indices);



    pcl::PointIndices::Ptr fInliers (new pcl::PointIndices);
    fInliers->indices=indices ;
    pcl::ExtractIndices<pcl::PointXYZ> extract ;
    extract.setInputCloud (scene_ptr);
    extract.setIndices (fInliers);
    extract.setNegative (true);
    extract.filter (*new_scene_ptr);

}

boost::filesystem::path RobotModel::resolve_path( const boost::filesystem::path& p, const boost::filesystem::path& base )
{
    boost::filesystem::path abs_p = boost::filesystem::absolute(p,base);
    boost::filesystem::path result;
    for(boost::filesystem::path::iterator it=abs_p.begin(); it!=abs_p.end(); ++it)
    {
        if(*it == "..")
        {
            // /a/b/.. is not necessarily /a if b is a symbolic link
            if(boost::filesystem::is_symlink(result) )
                result /= *it;
            // /a/b/../.. is not /a/b/.. under most circumstances
            // We can end up with ..s in our result because of symbolic links
            else if(result.filename() == "..")
                result /= *it;
            // Otherwise it should be safe to resolve the parent
            else
                result = result.parent_path();
        }
        else if(*it == ".")
        {
            // Ignore
        }
        else
        {
            // Just cat other path entries
            result /= *it;
        }
    }
    return result;
}

bool RobotModel::getJointLimits(std::vector< double > &lower_limits, std::vector< double > &upper_limits)
{
    std::vector< std::pair<std::string, urdf::Joint> > planning_groups_joints_names;
    std::string base_link, tip_link;
    if(!getPlanningGroupJointinformation(planning_group_name_, planning_groups_joints_names, base_link, tip_link))
	return false;
    //lower_limits.resize(planning_groups_joints_names.size());
    //upper_limits.resize(planning_groups_joints_names.size());
    lower_limits.clear();
    upper_limits.clear();
    
    
    for(auto it = planning_groups_joints_names.begin(); it != planning_groups_joints_names.end(); it++)
    {
        lower_limits.push_back(it->second.limits->lower)  ;
        upper_limits.push_back(it->second.limits->upper)  ;	
    }
    
    assert(lower_limits.size() == planning_groups_joints_names.size());
    
    return true;
}

/*

bool RobotModel::checkRangeBoundryForJointsFromIKFAST(std::vector<IkReal> joints_values_from_ik_fast)
{
    bool all_joints_are_in_range=true;
    for(std::size_t i=0;i<joints_values_from_ik_fast.size();i++)
    {
        if(  ( joints_values_from_ik_fast.at(i) < planningGroupJointsLowerBounds.at(i) ) ||   ( joints_values_from_ik_fast.at(i) > planningGroupJointsUpperBounds.at(i) ))
        {
            all_joints_are_in_range=false;
            break;
        }
    }
    return all_joints_are_in_range;
}

////////////////////////////////////////////////////////////IK Solver///////////////////////////////////////////////////////

PlannerStatus RobotModel::ikSolverUsingIKFAST(   const KDL::Frame & pose_in_chain_root,
                                        const std::string &planningGroupName ,
                                        std::map<std::string,double> &free_parameter_joints,
                                        std::vector< std::map<std::string,double> > &solutions_for_given_pose)
{

    //    #define IKSOKVERUSINGIKFAST
    manipulator_planner_library::PlannerStatus error_status;
    mPlanningGroupJointsName.clear();
    //    std::map<std::string,urdf::Joint> planning_group_joints;
    std::vector< std::pair<std::string,urdf::Joint> > planning_group_joints;
    std::string base_link, tip_link;
    this->getPlanningGroupJointinformation(planningGroupName, planning_group_joints, base_link, tip_link);

    for(std::vector< std::pair<std::string, urdf::Joint> >::iterator it= planning_group_joints.begin(); it!=planning_group_joints.end();it++)
    {
        mPlanningGroupJointsName.push_back(it->first);
    }

    IkReal eetrans[3];
    IkReal eerot[9];

    eetrans[0]=pose_in_chain_root.p.x();
    eetrans[1]=pose_in_chain_root.p.y();
    eetrans[2]=pose_in_chain_root.p.z();
    eerot[0]=pose_in_chain_root.M.data[0];
    eerot[1]=pose_in_chain_root.M.data[1];
    eerot[2]=pose_in_chain_root.M.data[2];
    eerot[3]=pose_in_chain_root.M.data[3];
    eerot[4]=pose_in_chain_root.M.data[4];
    eerot[5]=pose_in_chain_root.M.data[5];
    eerot[6]=pose_in_chain_root.M.data[6];
    eerot[7]=pose_in_chain_root.M.data[7];
    eerot[8]=pose_in_chain_root.M.data[8];


    std::vector<IkReal> freeParameter;
    double free_joint_parameter_value;

    for(std::map<std::string,double>::const_iterator  it=free_parameter_joints.begin();it!=free_parameter_joints.end();it++ )
    {
        free_joint_parameter_value=it->second;
        freeParameter.push_back(free_joint_parameter_value);
    }

    ikfast::IkSolutionList<IkReal> solutions;
//        bool bSuccess = IKFAST_NAMESPACE::ComputeIk(eetrans, eerot, &freeParameter[0], solutions);



    std::string abs_path_to_shared_object_file;
    abs_path_to_shared_object_file= this->getIKFASTSharedObjectAbsolutePath()+"/lib"+planningGroupName+"_ikfast.so";



    void *handle;
    bool (*ComputeIk_function_ptr)(const IkReal* , const IkReal* , const IkReal* , ikfast::IkSolutionListBase<IkReal>& );
    char *error;

    handle = dlopen (abs_path_to_shared_object_file.c_str(), RTLD_LAZY);
    if (!handle)
    {
        fputs (dlerror(), stderr);
        error_status.statuscode = PlannerStatus::NO_IK_SOLVER_FOUND;
        return error_status;
    }

//        IKFAST_NAMESPACE::ComputeIk
    ComputeIk_function_ptr=(  bool (*) (const IkReal* , const IkReal* , const IkReal* , ikfast::IkSolutionListBase<IkReal>& ) )  dlsym(handle, "ComputeIk");

    if ((error = dlerror()) != NULL)
    {
        fputs(error, stderr);
        dlclose(handle);
        error_status.statuscode = PlannerStatus::IK_SOLVER_FUNCTION_NOT_FOUND;
        return error_status;
    }

    bool ik_success =  ComputeIk_function_ptr  (eetrans, eerot, &freeParameter[0], solutions);

    if (ik_success)
    {

        solutions_for_given_pose.clear();
        std::vector<IkReal> ikfast_joint_values;
        std::vector<IkReal> freevalues;

        std::string joint_name;
        double joint_value;
        int joint_type;

        int none_fixed_joint=0;


        urdf::Joint urdf_joint;
        bool are_solution_joints_are_in_range=true;

        for(std::size_t i=0;i<solutions.GetNumSolutions();i++)
        {
            std::map<std::string,double> solution_joints_values;
            none_fixed_joint=0;
            solutions.GetSolution(i).GetSolution(ikfast_joint_values,freevalues);
            are_solution_joints_are_in_range=true;
            for(std::vector< std::pair<std::string, urdf::Joint> >::iterator it= planning_group_joints.begin(); it!=planning_group_joints.end();it++)
            {
                urdf_joint=it->second;
                joint_name=urdf_joint.name;
                joint_type=urdf_joint.type;


                if(joint_type!=urdf::Joint::FIXED)
                {
                    joint_value=ikfast_joint_values.at(none_fixed_joint);
                    if(joint_value< urdf_joint.limits->upper &&  joint_value>urdf_joint.limits->lower)
                    {
                        solution_joints_values[joint_name]=joint_value;
                        none_fixed_joint++;
                    }
                    else
                    {
                        are_solution_joints_are_in_range=false;
                        break;
                        #ifdef IKSOKVERUSINGIKFAST
                        std::cout<<"joint name is :"<<joint_name <<" joint lower is:"<< urdf_joint.limits->lower  <<" and joint value is: "<< joint_value<< " and joint upper is: "<< urdf_joint.limits->upper<< " not acceptable " <<std::endl;
                        #endif
                    }
                }
            }

            if(are_solution_joints_are_in_range)
            {
                solutions_for_given_pose.push_back( solution_joints_values);
            }
        }

        if(solutions_for_given_pose.size() > 0)
            error_status.statuscode = PlannerStatus::IK_SUCCESS;
        else
            error_status.statuscode = PlannerStatus::IK_SOLUTIONS_NOT_WITHIN_BOUNDS;

    }
    else
    {
        error_status.statuscode = PlannerStatus::NO_IK;
    }

    dlclose(handle);
    return error_status;

}

PlannerStatus RobotModel::ikSolverUsingIKFAST(   const KDL::Frame &pose_in_chain_root,
                                        const std::string &planningGroupName,
                                        std::vector<RobotFreeJointParameter> &robot_free_joint_parameters,
                                        std::map<std::string, double> &result)
{
  

//#define IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
    manipulator_planner_library::PlannerStatus error_status;
    mPlanningGroupJointsName.clear();
    this->previous_ik_exist=false;
    std::vector<IkReal> values_for_free_joints;
    int index=robot_free_joint_parameters.size()-1;
    this->IKFastSolution.clear();
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector< std::pair<std::string, urdf::Joint> > planning_groups_joints_name_from_base_to_tip;
    std::string base_link, tip_link;
    this->getPlanningGroupJointinformation(planningGroupName, planning_groups_joints_name_from_base_to_tip, base_link, tip_link);
    this->planningGroupJointsLowerBounds.clear();
    this->planningGroupJointsUpperBounds.clear();
    for(std::vector< std::pair<std::string, urdf::Joint> >::const_iterator it=planning_groups_joints_name_from_base_to_tip.begin();it!=planning_groups_joints_name_from_base_to_tip.end();it++)
    {
        this->planningGroupJointsLowerBounds.push_back(it->second.limits->lower)  ;
        this->planningGroupJointsUpperBounds.push_back(it->second.limits->upper)  ;
    }

    for(std::vector< std::pair<std::string, urdf::Joint> >::iterator it= planning_groups_joints_name_from_base_to_tip.begin(); it!=planning_groups_joints_name_from_base_to_tip.end();it++)
    {
        mPlanningGroupJointsName.push_back(it->first);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
        std::cout<< "[IKFAST]: Target Position: "<<pose_in_chain_root.p.x()<<" "<<pose_in_chain_root.p.y()<<" "<<pose_in_chain_root.p.z()<<std::endl;
    #endif

    this->eetrans[0]=pose_in_chain_root.p.x();
    this->eetrans[1]=pose_in_chain_root.p.y();
    this->eetrans[2]=pose_in_chain_root.p.z();
    this->eerot[0]=pose_in_chain_root.M.data[0];
    this->eerot[1]=pose_in_chain_root.M.data[1];
    this->eerot[2]=pose_in_chain_root.M.data[2];
    this->eerot[3]=pose_in_chain_root.M.data[3];
    this->eerot[4]=pose_in_chain_root.M.data[4];
    this->eerot[5]=pose_in_chain_root.M.data[5];
    this->eerot[6]=pose_in_chain_root.M.data[6];
    this->eerot[7]=pose_in_chain_root.M.data[7];
    this->eerot[8]=pose_in_chain_root.M.data[8];

    std::string abs_path_to_shared_object_file;
    abs_path_to_shared_object_file= this->getIKFASTSharedObjectAbsolutePath()+"/lib"+planningGroupName+"_ikfast.so";

    void *handle;

    char *error;

    handle = dlopen (abs_path_to_shared_object_file.c_str(), RTLD_LAZY);
    if (!handle)
    {
        fputs (dlerror(), stderr);
        error_status.statuscode = PlannerStatus::NO_IK_SOLVER_FOUND;
        return error_status;
    }


    this->ComputeIk_function_ptr=(  bool (*) (const IkReal* , const IkReal* , const IkReal* , ikfast::IkSolutionListBase<IkReal>& ) )  dlsym(handle, "ComputeIk");

    if ((error = dlerror()) != NULL)
    {
        fputs(error, stderr);
        dlclose(handle);
        error_status.statuscode = PlannerStatus::IK_SOLVER_FUNCTION_NOT_FOUND;
        return error_status;
    }


//////////////////////////////////////////////////////////////////////if we have more than 6DOF//////////////////////////////////////////////////////////////
    if(robot_free_joint_parameters.size()>0)
    {
        recursive_nested_loop_with_pivot(robot_free_joint_parameters, values_for_free_joints, index);
    }
//////////////////////////////////////////////////////////////////////if we have Exactly 6DOF//////////////////////////////////////////////////////////////
    else
    {
        this->solutions.Clear();
        bool ik_success =  this->ComputeIk_function_ptr  (this->eetrans, this->eerot, NULL, this->solutions);
        bool is_state_is_valid=false;
        bool are_joints_in_bounds=false;

        if(ik_success)
        {
            std::vector<IkReal> ikfast_joint_values;
            for(std::size_t i=0;i<this->solutions.GetNumSolutions();i++)
            {
                this->solutions.GetSolution(i).GetSolution(ikfast_joint_values,std::vector<IkReal>());
                are_joints_in_bounds=this->checkRangeBoundryForJointsFromIKFAST(ikfast_joint_values);
                if(are_joints_in_bounds)
                {
                    #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                    std::cout<<"one solution found, checking for collisions" <<std::endl;
                    #endif
                    //So far the we have a solution and joints angles are in bound now we have to check if the robot is not in the collision

                    std::vector<double> double_ikfast_joint_values(ikfast_joint_values.begin(),ikfast_joint_values.end() );

                   this->updateJointGroup( this->mPlanningGroupJointsName, double_ikfast_joint_values);
                   is_state_is_valid= this->IsStateIsValid();

                   if(is_state_is_valid)
                   {
                       #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                       std::cout<<"The joints are in bounds and the robot state is valid" <<std::endl;
                       #endif

                       #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                       std::cout<<"--------------------------------------------------------------------------" <<std::endl;
                       #endif
                       this->IKFastSolution=ikfast_joint_values;
                       break;
                   }
                   else
                   {
                        #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                        std::cout<<"The joints are in bounds but the robot is in collision" <<std::endl;
                        #endif
                   }

                }
                else
                {
                    #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                    std::cout<<"The joints are not in bounds" <<std::endl;
                    #endif
                }
            }
        }
        else
        {
            #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
            std::cout<<"No IK solution found for given pose" <<std::endl;
            #endif

            error_status.statuscode = PlannerStatus::NO_IK;
        }
    }

    if(this->IKFastSolution.size()>0)
    {
        for(std::size_t i= 0;i< planning_groups_joints_name_from_base_to_tip.size();i++)
        {
            std::string joint_name;
            joint_name=planning_groups_joints_name_from_base_to_tip.at(i).first;
            result[joint_name]=this->IKFastSolution.at(i);
            #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
            std::cout<<joint_name<<":"<<this->IKFastSolution.at(i) <<std::endl;
            #endif
        }

        error_status.statuscode = PlannerStatus::IK_SUCCESS;
    }
    else
    {
        #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
        std::cout<<"There is no IK solution" <<std::endl;
        #endif

        error_status.statuscode = PlannerStatus::NO_IK;
    }

    dlclose(handle);
    return error_status;
}

PlannerStatus RobotModel::ikSolverUsingIKFAST(	const std::map<std::string,double> & current_robot_status,   
												const KDL::Frame &pose_in_chain_root,
				                                const std::string &planningGroupName,
				                                std::vector<RobotFreeJointParameter> &robot_free_joint_parameters, 
				                                std::map<std::string, double> &result, double dummy)
{
  
    std::vector<manipulator_planner_library::sol_container> ikfast_sol_container_vec;

#define IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
    manipulator_planner_library::PlannerStatus error_status;
    mPlanningGroupJointsName.clear();
    this->previous_ik_exist=false;
    std::vector<IkReal> values_for_free_joints;
    int index=robot_free_joint_parameters.size()-1;
    this->IKFastSolution.clear();
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector< std::pair<std::string, urdf::Joint> > planning_groups_joints_name_from_base_to_tip;
    std::string base_link, tip_link;
    this->getPlanningGroupJointinformation(planningGroupName, planning_groups_joints_name_from_base_to_tip, base_link, tip_link);
    this->planningGroupJointsLowerBounds.clear();
    this->planningGroupJointsUpperBounds.clear();
    for(std::vector< std::pair<std::string, urdf::Joint> >::const_iterator it=planning_groups_joints_name_from_base_to_tip.begin();it!=planning_groups_joints_name_from_base_to_tip.end();it++)
    {
        this->planningGroupJointsLowerBounds.push_back(it->second.limits->lower)  ;
        this->planningGroupJointsUpperBounds.push_back(it->second.limits->upper)  ;
    }

    this->ikfast_joint_values_previous_result.clear();

    for(std::vector< std::pair<std::string, urdf::Joint> >::iterator it= planning_groups_joints_name_from_base_to_tip.begin(); it!=planning_groups_joints_name_from_base_to_tip.end();it++)
    {
        mPlanningGroupJointsName.push_back(it->first);
		this->ikfast_joint_values_previous_result.push_back(current_robot_status.find(it->first)->second);
    }

    if(mJointsWeight.empty())
        setDefaultJointWeight(mPlanningGroupJointsName);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
        std::cout<< "[IKFAST]: Target Position: "<<pose_in_chain_root.p.x()<<" "<<pose_in_chain_root.p.y()<<" "<<pose_in_chain_root.p.z()<<std::endl;
    #endif

    this->eetrans[0]=pose_in_chain_root.p.x();
    this->eetrans[1]=pose_in_chain_root.p.y();
    this->eetrans[2]=pose_in_chain_root.p.z();
    this->eerot[0]=pose_in_chain_root.M.data[0];
    this->eerot[1]=pose_in_chain_root.M.data[1];
    this->eerot[2]=pose_in_chain_root.M.data[2];
    this->eerot[3]=pose_in_chain_root.M.data[3];
    this->eerot[4]=pose_in_chain_root.M.data[4];
    this->eerot[5]=pose_in_chain_root.M.data[5];
    this->eerot[6]=pose_in_chain_root.M.data[6];
    this->eerot[7]=pose_in_chain_root.M.data[7];
    this->eerot[8]=pose_in_chain_root.M.data[8];

    std::string abs_path_to_shared_object_file;
    abs_path_to_shared_object_file= this->getIKFASTSharedObjectAbsolutePath()+"/lib"+planningGroupName+"_ikfast.so";

    void *handle;

    char *error;

    handle = dlopen (abs_path_to_shared_object_file.c_str(), RTLD_LAZY);
    if (!handle)
    {
        fputs (dlerror(), stderr);
        error_status.statuscode = PlannerStatus::NO_IK_SOLVER_FOUND;
        return error_status;
    }


    this->ComputeIk_function_ptr=(  bool (*) (const IkReal* , const IkReal* , const IkReal* , ikfast::IkSolutionListBase<IkReal>& ) )  dlsym(handle, "ComputeIk");

    if ((error = dlerror()) != NULL)
    {        
        fputs(error, stderr);
        dlclose(handle);
        error_status.statuscode = PlannerStatus::IK_SOLVER_FUNCTION_NOT_FOUND;
        return error_status;
    }
    


//////////////////////////////////////////////////////////////////////if we have more than 6DOF//////////////////////////////////////////////////////////////
    if(robot_free_joint_parameters.size()>0)
    {
        recursive_nested_loop_with_pivot(robot_free_joint_parameters, values_for_free_joints, index);
    }
//////////////////////////////////////////////////////////////////////if we have Exactly 6DOF//////////////////////////////////////////////////////////////
    else
    {
        this->solutions.Clear();
        bool ik_success =  this->ComputeIk_function_ptr  (this->eetrans, this->eerot, NULL, this->solutions);
        bool is_state_is_valid=false;
        bool are_joints_in_bounds=false;

        if(ik_success)
        {
            std::vector<IkReal> ikfast_joint_values;
            sol_container ikfast_sol_container;

            #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
            std::cout<<"IK Success and IKfast found "<<this->solutions.GetNumSolutions()<<" solution" <<std::endl;
            #endif
	    
            for(std::size_t i=0;i<this->solutions.GetNumSolutions();i++)
            {
                this->solutions.GetSolution(i).GetSolution(ikfast_joint_values,std::vector<IkReal>());
                are_joints_in_bounds=this->checkRangeBoundryForJointsFromIKFAST(ikfast_joint_values);
                if(are_joints_in_bounds)
                {
                    #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                    std::cout<<"one solution found, checking for collisions" <<std::endl;
                    #endif
                    //So far the we have a solution and joints angles are in bound now we have to check if the robot is not in the collision

                    std::vector<double> double_ikfast_joint_values(ikfast_joint_values.begin(),ikfast_joint_values.end() );

                   this->updateJointGroup( this->mPlanningGroupJointsName, double_ikfast_joint_values);
                   is_state_is_valid= this->IsStateIsValid();

                   if(is_state_is_valid)
                   {
                        #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                        std::cout<<"The joints are in bounds and the robot state is valid" <<std::endl;
                        #endif

                        #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                        std::cout<<"--------------------------------------------------------------------------" <<std::endl;
                        #endif
		    
		                ikfast_sol_container.ikfast_sol = ikfast_joint_values;
		                ikfast_sol_container.distance = calcDistanceBetweenJointsValues(this->ikfast_joint_values_previous_result, ikfast_joint_values, mJointsWeight);
		                ikfast_sol_container_vec.push_back(ikfast_sol_container);
                   }
                   else
                   {
                        #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                        std::cout<<"The joints are in bounds but the robot is in collision" <<std::endl;
                        #endif
                   }

                }
                else
                {
                    #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                    std::cout<<"The joints are not in bounds" <<std::endl;
                    #endif
                }
            }   
        }
        else
        {
            #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
            std::cout<<"No IK solution found for given pose" <<std::endl;
            #endif

            error_status.statuscode = PlannerStatus::NO_IK;
        }
    }
    
    #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
    std::cout<<"Number of valid solutions: " <<ikfast_sol_container_vec.size()<<std::endl;
    for(std::vector<manipulator_planner_library::sol_container>::iterator it = ikfast_sol_container_vec.begin(); it!=ikfast_sol_container_vec.end();it++)
    {        
        std::cout<<"Distance ="<<it->distance<<std::endl;
        for(int i = 0; i <6; i++)
            std::cout<<it->ikfast_sol.at(i)<<std::endl;
        std::cout<<" "<<std::endl;
    }
    std::cout<<"-----------------------"<<std::endl;
    #endif

    if(ikfast_sol_container_vec.size()>1)
    {
      std::sort(ikfast_sol_container_vec.begin(), ikfast_sol_container_vec.end(), compare_result());
    }
    
    if(ikfast_sol_container_vec.size()>0)
    {
      this->IKFastSolution = ikfast_sol_container_vec.at(0).ikfast_sol;
    }
    
      
    
    if(this->IKFastSolution.size()>0)
    {
        for(std::size_t i= 0;i< planning_groups_joints_name_from_base_to_tip.size();i++)
        {
            std::string joint_name;
            joint_name=planning_groups_joints_name_from_base_to_tip.at(i).first;
            result[joint_name]=this->IKFastSolution.at(i);
            #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
            std::cout<<joint_name<<":"<<this->IKFastSolution.at(i) <<std::endl;
            #endif
        }

        error_status.statuscode = PlannerStatus::IK_SUCCESS;
    }
    else
    {
        #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
        std::cout<<"There is no IK solution" <<std::endl;
        #endif

        error_status.statuscode = PlannerStatus::NO_IK;
    }

    dlclose(handle);
    return error_status;
}

PlannerStatus RobotModel::ikSolverUsingIKFAST(   const KDL::Frame &pose_in_chain_root,
                                        const std::string &planningGroupName,
                                        std::vector<RobotFreeJointParameter> &robot_free_joint_parameters,
                                        std::map<std::string, double> &previous_ik_result,double max_distance_alowed_between_joints_value,
                                        std::map<std::string, double> &result)
{
//#define IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
    manipulator_planner_library::PlannerStatus error_status;
    this->previous_ik_exist=true;
    mPlanningGroupJointsName.clear();

    std::vector<IkReal> values_for_free_joints;
    int index=robot_free_joint_parameters.size()-1;
    this->IKFastSolution.clear();
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector< std::pair<std::string, urdf::Joint> > planning_groups_joints_name_from_base_to_tip;
    std::string base_link, tip_link;
    this->getPlanningGroupJointinformation(planningGroupName, planning_groups_joints_name_from_base_to_tip, base_link, tip_link);
    this->planningGroupJointsLowerBounds.clear();
    this->planningGroupJointsUpperBounds.clear();
    for(std::vector< std::pair<std::string, urdf::Joint> >::const_iterator it=planning_groups_joints_name_from_base_to_tip.begin();it!=planning_groups_joints_name_from_base_to_tip.end();it++)
    {
        this->planningGroupJointsLowerBounds.push_back(it->second.limits->lower)  ;
        this->planningGroupJointsUpperBounds.push_back(it->second.limits->upper)  ;
    }


    this->max_distance_alowed_between_joints_value=max_distance_alowed_between_joints_value;


    for(std::vector< std::pair<std::string, urdf::Joint> >::iterator it= planning_groups_joints_name_from_base_to_tip.begin(); it!=planning_groups_joints_name_from_base_to_tip.end();it++)
    {
        mPlanningGroupJointsName.push_back(it->first);
    }

    this->ikfast_joint_values_previous_result.clear();
//previous_ik_result
    std::string joint_name;
    double joint_value;
    for(std::size_t i=0;i<mPlanningGroupJointsName.size();i++)
    {
        joint_name=mPlanningGroupJointsName.at(i);
        joint_value=previous_ik_result[joint_name];
        this->ikfast_joint_values_previous_result.push_back(joint_value);
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    this->eetrans[0]=pose_in_chain_root.p.x();
    this->eetrans[1]=pose_in_chain_root.p.y();
    this->eetrans[2]=pose_in_chain_root.p.z();
    this->eerot[0]=pose_in_chain_root.M.data[0];
    this->eerot[1]=pose_in_chain_root.M.data[1];
    this->eerot[2]=pose_in_chain_root.M.data[2];
    this->eerot[3]=pose_in_chain_root.M.data[3];
    this->eerot[4]=pose_in_chain_root.M.data[4];
    this->eerot[5]=pose_in_chain_root.M.data[5];
    this->eerot[6]=pose_in_chain_root.M.data[6];
    this->eerot[7]=pose_in_chain_root.M.data[7];
    this->eerot[8]=pose_in_chain_root.M.data[8];

    std::string abs_path_to_shared_object_file;
    abs_path_to_shared_object_file= this->getIKFASTSharedObjectAbsolutePath()+"/lib"+planningGroupName+"_ikfast.so";

    void *handle;

    char *error;

    handle = dlopen (abs_path_to_shared_object_file.c_str(), RTLD_LAZY);
    if (!handle)
    {
        fputs (dlerror(), stderr);
        error_status.statuscode = PlannerStatus::NO_IK_SOLVER_FOUND;
        return error_status;
    }


    this->ComputeIk_function_ptr=(  bool (*) (const IkReal* , const IkReal* , const IkReal* , ikfast::IkSolutionListBase<IkReal>& ) )  dlsym(handle, "ComputeIk");

    if ((error = dlerror()) != NULL)
    {
        fputs(error, stderr);
        dlclose(handle);
        error_status.statuscode = PlannerStatus::IK_SOLVER_FUNCTION_NOT_FOUND;
        return error_status;
    }


//////////////////////////////////////////////////////////////////////if we have more than 6DOF//////////////////////////////////////////////////////////////
    if(robot_free_joint_parameters.size()>0)
    {
        recursive_nested_loop_with_pivot(robot_free_joint_parameters, values_for_free_joints, index);
    }
//////////////////////////////////////////////////////////////////////if we have Exactly 6DOF//////////////////////////////////////////////////////////////
    else
    {
        this->solutions.Clear();
        bool ik_success =  this->ComputeIk_function_ptr  (this->eetrans, this->eerot, NULL, this->solutions);
        bool is_state_is_valid=false;
        bool are_joints_in_bounds=false;

        if(ik_success)
        {
            std::vector<IkReal> ikfast_joint_values;
            for(std::size_t i=0;i<this->solutions.GetNumSolutions();i++)
            {
                this->solutions.GetSolution(i).GetSolution(ikfast_joint_values,std::vector<IkReal>());
                are_joints_in_bounds=this->checkRangeBoundryForJointsFromIKFAST(ikfast_joint_values);
                if(are_joints_in_bounds)
                {
                    #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                    std::cout<<"one solution found, checking for collisions" <<std::endl;
                    #endif
                    //So far the we have a solution and joints angles are in bound now we have to check if the robot is not in the collision

                    std::vector<double> double_ikfast_joint_values(ikfast_joint_values.begin(),ikfast_joint_values.end() );

                   this->updateJointGroup( this->mPlanningGroupJointsName, double_ikfast_joint_values);
                   is_state_is_valid= this->IsStateIsValid();

                   if(is_state_is_valid)
                   {
                       #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                       std::cout<<"The joints are in bounds and the robot state is valid" <<std::endl;
                       #endif

                       #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                       std::cout<<"--------------------------------------------------------------------------" <<std::endl;
                       #endif
                       this->IKFastSolution=ikfast_joint_values;
                       break;
                   }
                   else
                   {
                        #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                        std::cout<<"The joints are in bounds but the robot is in collision" <<std::endl;
                        #endif
                   }
                }
                else
                {
                    #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
                    std::cout<<"The joints are not in bounds" <<std::endl;
                    #endif
                }
            }
        }
        else
        {
            #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
            std::cout<<"No IK solution found for given pose" <<std::endl;
            #endif

            error_status.statuscode = PlannerStatus::NO_IK;
        }
    }

    ///////////////////////////////////////////////////////////////////////if  the result of IK was successfull////////////////////////////////////////////////////////////////////

    if(this->IKFastSolution.size()>0)
    {
        for(std::size_t i= 0;i< planning_groups_joints_name_from_base_to_tip.size();i++)
        {
            std::string joint_name;
            joint_name=planning_groups_joints_name_from_base_to_tip.at(i).first;
            result[joint_name]=this->IKFastSolution.at(i);
            #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
            std::cout<<joint_name<<":"<<this->IKFastSolution.at(i) <<std::endl;
            #endif
        }

        error_status.statuscode = PlannerStatus::IK_SUCCESS;
    }
    else
    {
        #ifdef IKSOLVERUSINGIKFASTBRUTEFORCE_LOG
        std::cout<<"There is no IK solution" <<std::endl;
        #endif

        error_status.statuscode = PlannerStatus::NO_IK;
    }

    dlclose(handle);
    return error_status;
}

bool RobotModel::FindIKSolution(std::vector<RobotFreeJointParameter> &robot_free_joint_parameters, std::vector<IkReal> &values_for_free_joints,IkReal free_joint_parameter_values,int index )
{

// free_joint_parameter_values is pivot+step or pivot-step

    bool result_of_ik=false;
//    #define FINDIKSOLUTION_LOG
    #ifdef FINDIKSOLUTION_LOG
    std::cout<<"Setting the followng values for free joint parameters and chekcing IK" <<std::endl;

    int negate =values_for_free_joints.size();

    for(std::size_t j=0;j<values_for_free_joints.size();j++)
    {
        std::cout<<"joint name: " <<robot_free_joint_parameters.at( negate- j).getJointName()<<" " <<values_for_free_joints.at(j) <<", ";
    }
    std::cout<<"joint name: " <<robot_free_joint_parameters.at(index).getJointName()<<" "  << free_joint_parameter_values<< std::endl;
    #endif

    //Here we try to find the ik solution
    values_for_free_joints.push_back(free_joint_parameter_values);
    this->solutions.Clear();
    bool ik_success =  this->ComputeIk_function_ptr  (this->eetrans, this->eerot, &values_for_free_joints[0], this->solutions);
    //We have found some solutions for given, now we have to check if the joint angles are in bounds
    if(ik_success)
    {
        std::vector<IkReal> ikfast_joint_values;
        std::vector<IkReal> freevalues;
        for(std::size_t i=0;i<this->solutions.GetNumSolutions();i++)
        {
            this->solutions.GetSolution(i).GetSolution(ikfast_joint_values,freevalues);
            if(this->checkRangeBoundryForJointsFromIKFAST(ikfast_joint_values))
            {
                #ifdef FINDIKSOLUTION_LOG
                std::cout<<"one solution found, checking for collisions" <<std::endl;
                #endif
                //So far the we have a colution and joints angles are in bound now we have to check if the robot is not in the collision

                std::vector<double> double_ikfast_joint_values(ikfast_joint_values.begin(),ikfast_joint_values.end() );

               this->updateJointGroup( this->mPlanningGroupJointsName, double_ikfast_joint_values);
               if(this->IsStateIsValid())
               {
                   #ifdef FINDIKSOLUTION_LOG
                   std::cout<<"The joints are in bounds and the robot state is valid" <<std::endl;
                   #endif
                   if(this->previous_ik_exist)
                   {
                        if(distance_between_joints_values(this->ikfast_joint_values_previous_result, ikfast_joint_values,this->max_distance_alowed_between_joints_value))
                        {
                            #ifdef FINDIKSOLUTION_LOG
                            std::cout<<"The joints are in bounds and the robot state is valid, and the joint distances are close enough to previus values" <<std::endl;
                            #endif
                            IKFastSolution=ikfast_joint_values;
                            result_of_ik= true;
                             #ifdef FINDIKSOLUTION_LOG
                             std::cout<<"--------------------------------------------------------------------------" <<std::endl;
                             #endif
                            return result_of_ik;

                        }
                        else
                        {
                            #ifdef FINDIKSOLUTION_LOG
                            std::cout<<"The joints are in bounds and the robot state is valid, but the joint distances are far away from previus values" <<std::endl;
                            #endif
                        }
                   }
                   else
                   {
                       IKFastSolution=ikfast_joint_values;
                       result_of_ik= true;
                        #ifdef FINDIKSOLUTION_LOG
                        std::cout<<"--------------------------------------------------------------------------" <<std::endl;
                        #endif
                       return result_of_ik;
                   }
               }
               else
               {
                    #ifdef FINDIKSOLUTION_LOG
                    std::cout<<"The joints are in bounds but the robot is in collision" <<std::endl;
                    #endif
               }

            }
        }
    }
    else
    {
        #ifdef FINDIKSOLUTION_LOG
        std::cout<<"No IK solution for given joint values for free joints" <<std::endl;
        #endif

    }
    #ifdef FINDIKSOLUTION_LOG
    std::cout<<"--------------------------------------------------------------------------" <<std::endl;
    #endif

    values_for_free_joints.pop_back();
    return result_of_ik;

}

bool RobotModel::distance_between_joints_values(std::vector<IkReal> ikfast_joint_values_previous_result,std::vector<IkReal> ikfast_joint_values_from_ik_fast_for_current_pose,double  max_distance_alowed_between_joints_value)
{
    double distance;
    for(std::size_t i=0;i<ikfast_joint_values_previous_result.size();i++)
    {
        distance=std::abs(ikfast_joint_values_previous_result.at(i)-ikfast_joint_values_from_ik_fast_for_current_pose.at(i));

        if(distance>max_distance_alowed_between_joints_value )
        {
            return false;
        }
    }
    return true;
}

double RobotModel::calcDistanceBetweenJointsValues( std::vector<IkReal> current_joint_angles, std::vector<IkReal> ik_solution, 
                                                    const std::vector<manipulator_planner_library::JointWeight>& jt_weight)
{
    double distance =0;
    //std::cout<<"current_joint_angles.size "<<current_joint_angles.size()<<std::endl;
    for(std::size_t i=0;i<current_joint_angles.size();i++)
    {
        //std::cout<<"Joint "<<i+1<<" Previo = "<<current_joint_angles.at(i) <<" solu="<<ik_solution.at(i)<<std::endl;
        distance= distance + jt_weight.at(i).weight * (std::abs(current_joint_angles.at(i)-ik_solution.at(i)));
     
    }
    //std::cout<<"  "<<std::endl;
    return distance;
}

bool RobotModel::recursive_nested_loop_with_pivot(std::vector<RobotFreeJointParameter> &robot_free_joint_parameters  ,std::vector<IkReal> &values_for_free_joints, int index)
{
#define RECURSIVE_NESTED_LOOP_WITH_PIVOT
    if(index>0)
    {
        if(robot_free_joint_parameters.at(index).jointPositionHasBeenSet())
        {
            double joint_position=robot_free_joint_parameters.at(index).getJointPosition();

            values_for_free_joints.push_back( joint_position);
            if( recursive_nested_loop_with_pivot(robot_free_joint_parameters, values_for_free_joints, index-1) )
            {
                return true;
            }
            values_for_free_joints.pop_back();

        }
        else
        {
            bool lower_boundry, upper_boundry;

            lower_boundry=true;
            upper_boundry=true;
            double pivot;

            double current_lower_bound=robot_free_joint_parameters.at(index).getLowerLimit();
            double current_upper_bound =robot_free_joint_parameters.at(index).getUpperLimit();

            pivot=robot_free_joint_parameters.at(index).getPivot();

            double step=0;
            double step_size=robot_free_joint_parameters.at(index).getStepSize();


            while(lower_boundry || upper_boundry)
            {
                step=step+step_size;
                if(current_upper_bound < pivot+step)
                {
                    upper_boundry=false;
                }
                else
                {
                    values_for_free_joints.push_back( pivot+step);
                    if( recursive_nested_loop_with_pivot(robot_free_joint_parameters, values_for_free_joints, index-1) )
                    {
                        return true;
                    }
                    values_for_free_joints.pop_back();
                }
                if(pivot-step < current_lower_bound)
                {
                    lower_boundry=false;
                }
                else
                {
                    values_for_free_joints.push_back(pivot-step);
                    if(recursive_nested_loop_with_pivot(robot_free_joint_parameters, values_for_free_joints, index-1))
                    {
                        return true;
                    }
                    values_for_free_joints.pop_back();
                }
            }
        }
    }
    else
    {
        if(robot_free_joint_parameters.at(index).jointPositionHasBeenSet())
        {
            double joint_position=robot_free_joint_parameters.at(index).getJointPosition();

            if(this->FindIKSolution(robot_free_joint_parameters,values_for_free_joints,joint_position, index))
            {
                return true;
            }
        }
        else
        {
            bool lower_boundry, upper_boundry;

            lower_boundry=true;
            upper_boundry=true;
            double pivot;

            double current_lower_bound=robot_free_joint_parameters.at(index).getLowerLimit();
            double current_upper_bound =robot_free_joint_parameters.at(index).getUpperLimit();
            pivot=robot_free_joint_parameters.at(index).getPivot();
            double step=0;
            double step_size=robot_free_joint_parameters.at(index).getStepSize();


            while(lower_boundry || upper_boundry)
            {
                step=step+step_size;
                if(current_upper_bound < pivot+step)
                {
                    upper_boundry=false;
                }
                else
                {
                    if(this->FindIKSolution(robot_free_joint_parameters,values_for_free_joints,pivot+step, index))
                    {
                        return true;
                    }
                }

                if(pivot-step < current_lower_bound)
                {
                    lower_boundry=false;
                }
                else
                {
                    if(this->FindIKSolution(robot_free_joint_parameters,values_for_free_joints,pivot-step, index))
                    {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}
*/

/*
bool RobotModel::ikSolverUsingKDL(const KDL::Frame & pose_in_chain_root,std::string  &planningGroupName , std::map<std::string,double>  &solution_for_given_pose, unsigned int 	maxiter,  double  eps )
{
    #define IKSOLVERUSINGKDL_LOG


    std::map<std::string, urdf::Joint> planning_group_joints;

    std::string chain_root;
    std::string chain_tip;

    this->getPlanningGroupJointinformation(planningGroupName, planning_group_joints,chain_root,chain_tip);

    std::map<std::string, double > q_min_map;
    std::map<std::string, double > q_max_map;
    KDL::JntArray q_min;
    KDL::JntArray q_max;

    std::string joint_name;

    q_min.resize( planning_group_joints.size() );
    q_max.resize( planning_group_joints.size() );

    for(std::map<std::string, urdf::Joint>::iterator it=planning_group_joints.begin();it!=planning_group_joints.end();  it++)
    {
        joint_name=it->first;
        q_min_map[joint_name]=it->second.limits->lower;
        q_max_map[joint_name]=it->second.limits->upper;
        #ifdef    IKSOLVERUSINGKDL_LOG
        #endif
    }






    KDL::Chain 	kdl_chain;



    kdl_tree_.getChain(chain_root, chain_tip , kdl_chain);

     KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);




     for(std::size_t i=0;i<kdl_chain.segments.size();i++ )
     {
         //KDL JointType: RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,None;
         if(! (kdl_chain_.getSegment(i).getJoint().getType()==KDL::Joint::None) )
         {
             joint_name=kdl_chain_.getSegment(i).getJoint().getName();
//             planning_group_joint=*(urdf_model_->getJoint(joint_name).get());
//             planning_groups_joints[joint_name]=planning_group_joint;
             q_min.data[i]=q_min_map[joint_name];
             q_max.data[i]=q_max_map[joint_name];
         }
     }







//  inverse velocity kinematics algorithm based  on the generalize pseudo inverse
    KDL::ChainIkSolverVel_pinv  ik_solver_vel ( kdl_chain);
//        KDL::ChainIkSolverVel_pinv_givens ik_solver_vel( kdl_chain);
//        std::cout<<"using KDL::ChainIkSolverVel_pinv_givens "<<std::endl;
//        KDL::ChainIkSolverVel_wdls ik_solver_vel( kdl_chain);
//        KDL::ChainIkSolverVel_pinv_nso ik_solver_vel( kdl_chain);




    KDL::ChainIkSolverPos_NR_JL ik_solver_pos_with_joint_limits(kdl_chain, q_min, q_max , fk_solver, ik_solver_vel , maxiter, eps);
//        KDL::ChainIkSolverPos_NR ik_solver_pos_without_joint_limits( kdl_chain , fk_solver, ik_solver_vel,maxiter,eps);




     KDL::JntArray joint_values_from_ik;
     joint_values_from_ik.resize( q_min.rows()  );
     for(std::size_t i=0;i<joint_values_from_ik.rows();i++)
     {
        joint_values_from_ik.data[i]=(q_min.data[i] + q_max.data[i])/2;
        #ifdef    IKSOLVERUSINGKDL_LOG

        #endif
     }

    int result = ik_solver_pos_with_joint_limits.CartToJnt(joint_values_from_ik,pose_in_chain_root, joint_values_from_ik ) ;
//        int result =ik_solver_pos_without_joint_limits.CartToJnt(joint_values_from_ik,pose_in_chain_root, joint_values_from_ik ) ;

    double joint_value;

    if(result<0)
    {
        return false;
    }
    else
    {
//        for(std::size_t i=0; i<planning_group_joints.size() ;i++)
//        {
//            if(planning_group_joints.at(i).type != urdf::Joint::FIXED)
//            {
//                joint_name= planning_group_joints.at(i).name;
//                joint_value= joint_values_from_ik.data[i];
//                solution_for_given_pose[joint_name]=joint_value;
//            }
//        }


    int i=0;
    for(std::map<std::string, urdf::Joint>::iterator it=planning_group_joints.begin();it!=planning_group_joints.end(); it++)
    {
        if(it->second.type != urdf::Joint::FIXED)
        {
            joint_name= it->second.name;
            joint_value= joint_values_from_ik.data[i];
            solution_for_given_pose[joint_name]=joint_value;
            i++;
        }
    }


        return true;
    }
}
*/

/*

/////////////////////////////////////////////////////////FK Solver ////////////////////////////////////////////////////////////////
void RobotModel::fkSolverUsingIKFAST(const std::map<std::string,double> &joints_and_names, const std::string  &planningGroupName,KDL::Frame &tip_link_frame_pose_in_chain_root )
{
    std::string base_link;
    std::string tip_link;
    std::vector<IkReal> joint_values;

    std::string abs_path_to_shared_object_file;
    abs_path_to_shared_object_file= this->getIKFASTSharedObjectAbsolutePath()+"/lib"+planningGroupName+"_ikfast.so";

    void *handle;
    void (*ComputeFk_function_ptr)(const IkReal* , IkReal* , IkReal* );
    char *error;


    handle = dlopen (abs_path_to_shared_object_file.c_str(), RTLD_LAZY);
    if (!handle)
    {
        fputs (dlerror(), stderr);
        exit(1);
    }


//    std::vector<std::string>   planning_groups_joints_name_from_base_to_tip;
    std::vector<std::pair<std::string,urdf::Joint  > >   planning_groups_joints_name_from_base_to_tip;

    this->getPlanningGroupJointinformation(planningGroupName  ,  planning_groups_joints_name_from_base_to_tip, base_link,  tip_link);

    //for(std::size_t i=0;i<planning_groups_joints_name_from_base_to_tip.size();i++)
    //{
    //        joint_name=planning_groups_joints_name_from_base_to_tip.at(i).first;
    //        const double joint_value=joints_and_names[joint_name];
    //        joint_values.push_back(joint_value);
    //}
    
    std::map<std::string,double>::const_iterator current_status_it;

    for(std::size_t i=0;i<planning_groups_joints_name_from_base_to_tip.size();i++)
    {

        current_status_it = joints_and_names.find(planning_groups_joints_name_from_base_to_tip.at(i).first);

        if(current_status_it != joints_and_names.end())
        {
            joint_values.push_back(joints_and_names.find(planning_groups_joints_name_from_base_to_tip.at(i).first)->second);
        }
        else
        {
            throw std::out_of_range("trying to access element " + planning_groups_joints_name_from_base_to_tip.at(i).first +
                                    ", but there is no element with that name on this map");
        }
    }

    ComputeFk_function_ptr=(  void (*) ( const IkReal* , IkReal* , IkReal* ) )  dlsym(handle, "ComputeFk");

    if ((error = dlerror()) != NULL)
    {
        fputs(error, stderr);
        exit(1);
    }

    IkReal eetrans[3] = {0.0, 0.0, 0.0};
    IkReal eerot[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    ComputeFk_function_ptr( &joint_values[0], eetrans, eerot);

    KDL::Rotation r6( eerot[0],eerot[1],eerot[2],eerot[3],eerot[4],eerot[5],eerot[6],eerot[7],eerot[8]);
    KDL::Vector v3(eetrans[0],eetrans[1],eetrans[2]);

    tip_link_frame_pose_in_chain_root=KDL::Frame(r6,v3);

}


void RobotModel::fkSolverUsingKDL(const std::string &chain_root_link,const  std::string& tip_link, std::map<std::string, double> joints_name_values,KDL::Frame &tip_link_frame_pose_in_chain_root)
{

    kdl_tree_.getChain(chain_root_link , tip_link , kdl_chain_);

    std::string joint_name;
    double joint_value;
    KDL::JntArray kdl_chain_joint_array;
    KDL::Joint::JointType joint_type;
    int j=0;
    kdl_chain_joint_array.resize(joints_name_values.size() );
    for(std::size_t i=0;i<kdl_chain_.getNrOfSegments();i++ )
    {
        joint_name=kdl_chain_.getSegment(i).getJoint().getName();
        joint_type= kdl_chain_.getSegment(i).getJoint().getType();
        joint_value=joints_name_values[joint_name];
        if(joint_type !=KDL::Joint::None )
        {
            kdl_chain_joint_array.data[j]=joint_value;
            j++;
        }

    }

    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
    fk_solver.JntToCart(kdl_chain_joint_array, tip_link_frame_pose_in_chain_root);
}

*/

/*

Manipulability Analysis

http://h2t.anthropomatik.kit.edu/pdf/Vahrenkamp2012c.pdf

With Yoshikawa’s manipulability index [3] a quality measure for redundant manipulators was introduced, which de-
scribes the distance to singular configurations.

*/

void RobotModel::manipulabilityIndex(KDL::Jacobian  &jacobian, double &manipulability_index)
{
    Eigen::Matrix<double,6,6> jacobian_multiple_by_jacobian_inverse=jacobian.data*jacobian.data.transpose();
    manipulability_index=jacobian_multiple_by_jacobian_inverse.determinant();
}


void RobotModel::computeJacobain(const std::string &chain_root_link,const  std::string& tip_link, std::map<std::string, double> joints_name_values, KDL::Jacobian  &jacobian)
{
//#define COMPUTEJACOBAIN_LOG
    KDL::Chain kdl_chain;
    kdl_tree_.getChain(chain_root_link , tip_link , kdl_chain);
    std::string joint_name;
    double joint_value;
    KDL::JntArray kdl_chain_joint_array;
    KDL::Joint::JointType joint_type;
    int j=0;
    kdl_chain_joint_array.resize(joints_name_values.size() );
    for(std::size_t i=0;i<kdl_chain.getNrOfSegments();i++ )
    {
        joint_name=kdl_chain.getSegment(i).getJoint().getName();
        joint_type= kdl_chain.getSegment(i).getJoint().getType();
        joint_value=joints_name_values[joint_name];
        if(joint_type !=KDL::Joint::None )
        {
            kdl_chain_joint_array.data[j]=joint_value;
            j++;
        }

    }
//    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_(new KDL::ChainJntToJacSolver(kdl_chain_));

    jacobian.resize(j);

    KDL::ChainJntToJacSolver jnt_to_jac_solver_(kdl_chain);
    jnt_to_jac_solver_.JntToJac( kdl_chain_joint_array, jacobian);

    #ifdef COMPUTEJACOBAIN_LOG
    for(std::size_t i=0;i<jacobian.rows();i++ )
    {
        for(std::size_t j=0;j<jacobian.columns();j++)
        {
            std::cout<<"jacobian(i,j): " << jacobian(i,j)<<std::endl;
        }
    }
    #endif
}

bool RobotModel::getPlanningGroupJointinformation(const std::string planningGroupName,
                                                  std::vector< std::pair<std::string,urdf::Joint> > &planning_groups_joints,
                                                  std::string &base_link, std::string &tip_link)
{
    //    planning_groups_joints are in  order from base to tip! very important

    std::vector<srdf::Model::Group> srdf_groups= this->srdf_model_->getGroups();
    srdf::Model::Group planning_group;

    for(std::size_t i=0;i<srdf_groups.size();i++)
    {
        if( planningGroupName.compare(srdf_groups.at(i).name_) ==0 )
        {
            planning_group=srdf_groups.at(i);
            break;
        }
    }

    std::string joint_name;

    for(std::size_t i=0;i<planning_group.chains_.size();i++)
    {
        base_link=planning_group.chains_.at(i).first;
        tip_link=planning_group.chains_.at(i).second;
    }

    if(!kdl_tree_.getChain(base_link, tip_link , kdl_chain_))
	return false;

    urdf::Joint planning_group_joint;

    for(std::size_t i=0;i<kdl_chain_.segments.size();i++ )
    {
        //KDL JointType: RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,None;
        if(! (kdl_chain_.getSegment(i).getJoint().getType()==KDL::Joint::None) )
        {
            joint_name=kdl_chain_.getSegment(i).getJoint().getName();
            planning_group_joint=*(urdf_model_->getJoint(joint_name).get());

            planning_groups_joints.push_back(std::make_pair(joint_name ,planning_group_joint )   );
        }
    }
    
    return true;
}

void RobotModel::getPlanningGroupJointsName(const std::string planningGroupName,
                                            std::vector< std::string> &planning_group_joints_name)
{
    //    planning_groups_joints are in  order from base to tip! very important
    std::string base_link, tip_link;
    std::vector< std::pair<std::string,urdf::Joint> > planning_group_joints;

    getPlanningGroupJointinformation(planningGroupName, planning_group_joints, base_link, tip_link);

    planning_group_joints_name.clear();

    for(std::vector< std::pair<std::string, urdf::Joint> >::iterator it= planning_group_joints.begin(); it!=planning_group_joints.end();it++)    
        planning_group_joints_name.push_back(it->first);
}

void RobotModel::setSRDF(boost::shared_ptr<srdf::Model> &srdf_model_)
{
    this->srdf_model_=srdf_model_;
}

void RobotModel::setURDF(urdf::ModelInterfaceSharedPtr &urdf_model_)
{
    this->urdf_model_=urdf_model_;
}

boost::shared_ptr<srdf::Model>  const & RobotModel::getSRDF()
{
    return this->srdf_model_;
}

urdf::ModelInterfaceSharedPtr const &  RobotModel::getURDF()
{
    return this->urdf_model_;
}


void RobotModel::createPointCloudFromCollision(std::vector<urdf::CollisionSharedPtr > &link_collisions, std::vector<pcl::PointCloud<pcl::PointXYZ> > &link_point_cloud )
{
//    #define CREATEPOINTCLOUDFROMVISUAL_LOG
    boost::filesystem::path relative_path_of_mesh_file_in_urdf_file;
    boost::filesystem::path urdf_abs_path;
    boost::filesystem::path abs_path_of_mesh_file;
    std::string urdf_directory_path;

    std::string abs_path_to_mesh_file;
    double radius, length ,x,y,z, scale_for_mesha_files_x, scale_for_mesha_files_y , scale_for_mesha_files_z ;

    std::shared_ptr<collision_detection::MeshLoader> mesh_loader(new collision_detection::MeshLoader());
    

    for(std::size_t i=0;i<link_collisions.size();i++ )
    {
        if(link_collisions.at(i)->geometry->type == urdf::Geometry::MESH)
        {
            #ifdef CREATEPOINTCLOUDFROMVISUAL_LOG
            std::cout<<"------------------------------registering mesh file------------------------------ " <<std::endl;
            #endif
            urdf::MeshSharedPtr urdf_mesh_ptr= urdf::static_pointer_cast <urdf::Mesh> (link_collisions.at(i)->geometry);
            scale_for_mesha_files_x=urdf_mesh_ptr->scale.x;
            scale_for_mesha_files_y=urdf_mesh_ptr->scale.y;
            scale_for_mesha_files_z=urdf_mesh_ptr->scale.z;

            relative_path_of_mesh_file_in_urdf_file=urdf_mesh_ptr->filename;
            urdf_abs_path=urdf_file_abs_path_;



            urdf_directory_path=urdf_file_abs_path_.substr(0, urdf_file_abs_path_.find_last_of("/") );

            abs_path_of_mesh_file =resolve_path( relative_path_of_mesh_file_in_urdf_file, urdf_directory_path );

            abs_path_to_mesh_file=abs_path_of_mesh_file.string();
            pcl::PointCloud<pcl::PointXYZ> point_cloud;

            mesh_loader->createPointCloudFromMesh(abs_path_to_mesh_file, point_cloud, scale_for_mesha_files_x,scale_for_mesha_files_y,scale_for_mesha_files_z);
            link_point_cloud.push_back(point_cloud);



        }
        else if(link_collisions.at(i)->geometry->type == urdf::Geometry::BOX)
        {

            #ifdef CREATEPOINTCLOUDFROMVISUAL_LOG
              std::cout<<"------------------------------registering box------------------------------ " <<std::endl;
            #endif

            urdf::BoxSharedPtr urdf_box_ptr= urdf::static_pointer_cast <urdf::Box> (link_collisions.at(i)->geometry);
            x=urdf_box_ptr->dim.x;
            y=urdf_box_ptr->dim.y;
            z=urdf_box_ptr->dim.z;
            pcl::PointCloud<pcl::PointXYZ> point_cloud;

            createPtCloudFromBox(point_cloud,x,y,z);
            link_point_cloud.push_back(point_cloud);


        }
        else if(link_collisions.at(i)->geometry->type == urdf::Geometry::CYLINDER)
        {

            #ifdef CREATEPOINTCLOUDFROMVISUAL_LOG
            std::cout<<"------------------------------registering cylinder------------------------------" <<std::endl;
            #endif

            urdf::CylinderSharedPtr urdf_cylinder_ptr= urdf::static_pointer_cast <urdf::Cylinder> (link_collisions.at(i)->geometry);
            radius=urdf_cylinder_ptr->radius;
            length=urdf_cylinder_ptr->length;

            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            createPtCloudFromCylinder(point_cloud,radius,length);
            link_point_cloud.push_back(point_cloud);
        }
        else if(link_collisions.at(i)->geometry->type == urdf::Geometry::SPHERE)
        {

            #ifdef CREATEPOINTCLOUDFROMVISUAL_LOG
              std::cout<<"------------------------------registering sphere------------------------------ " <<std::endl;
            #endif

            urdf::SphereSharedPtr urdf_sphere_ptr= urdf::static_pointer_cast <urdf::Sphere> (link_collisions.at(i)->geometry);
            radius=urdf_sphere_ptr->radius;
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            createPtCloudFromSphere(point_cloud,radius);
            link_point_cloud.push_back(point_cloud);
        }
    }
}


void RobotModel::createPointCloudFromVisual(std::vector<urdf::VisualSharedPtr > &link_visuals, std::vector<pcl::PointCloud<pcl::PointXYZ> > &link_point_cloud )
{
//    #define CREATEPOINTCLOUDFROMVISUAL_LOG
    boost::filesystem::path relative_path_of_mesh_file_in_urdf_file;
    boost::filesystem::path urdf_abs_path;
    boost::filesystem::path abs_path_of_mesh_file;
    std::string urdf_directory_path;

    std::string abs_path_to_mesh_file;
    double radius, length ,x,y,z, scale_for_mesha_files_x, scale_for_mesha_files_y , scale_for_mesha_files_z ;

    std::shared_ptr<collision_detection::MeshLoader> mesh_loader(new collision_detection::MeshLoader());

    for(std::size_t i=0;i<link_visuals.size();i++ )
    {
        if(link_visuals.at(i)->geometry->type == urdf::Geometry::MESH)
        {
            #ifdef CREATEPOINTCLOUDFROMVISUAL_LOG
            std::cout<<"------------------------------registering mesh file------------------------------ " <<std::endl;
            #endif
            urdf::MeshSharedPtr urdf_mesh_ptr= urdf::static_pointer_cast <urdf::Mesh> (link_visuals.at(i)->geometry);
            scale_for_mesha_files_x=urdf_mesh_ptr->scale.x;
            scale_for_mesha_files_y=urdf_mesh_ptr->scale.y;
            scale_for_mesha_files_z=urdf_mesh_ptr->scale.z;

            relative_path_of_mesh_file_in_urdf_file=urdf_mesh_ptr->filename;
            urdf_abs_path=urdf_file_abs_path_;



            urdf_directory_path=urdf_file_abs_path_.substr(0, urdf_file_abs_path_.find_last_of("/") );

            abs_path_of_mesh_file =resolve_path( relative_path_of_mesh_file_in_urdf_file, urdf_directory_path );

            abs_path_to_mesh_file=abs_path_of_mesh_file.string();
            pcl::PointCloud<pcl::PointXYZ> point_cloud;

            mesh_loader->createPointCloudFromMesh(abs_path_to_mesh_file, point_cloud, scale_for_mesha_files_x,scale_for_mesha_files_y,scale_for_mesha_files_z);
            link_point_cloud.push_back(point_cloud);



        }
        else if(link_visuals.at(i)->geometry->type == urdf::Geometry::BOX)
        {

            #ifdef CREATEPOINTCLOUDFROMVISUAL_LOG
              std::cout<<"------------------------------registering box------------------------------ " <<std::endl;
            #endif

            urdf::BoxSharedPtr urdf_box_ptr= urdf::static_pointer_cast <urdf::Box> (link_visuals.at(i)->geometry);
            x=urdf_box_ptr->dim.x;
            y=urdf_box_ptr->dim.y;
            z=urdf_box_ptr->dim.z;
            pcl::PointCloud<pcl::PointXYZ> point_cloud;

            createPtCloudFromBox(point_cloud,x,y,z);
            link_point_cloud.push_back(point_cloud);


        }
        else if(link_visuals.at(i)->geometry->type == urdf::Geometry::CYLINDER)
        {

            #ifdef CREATEPOINTCLOUDFROMVISUAL_LOG
            std::cout<<"------------------------------registering cylinder------------------------------" <<std::endl;
            #endif

            urdf::CylinderSharedPtr urdf_cylinder_ptr= urdf::static_pointer_cast <urdf::Cylinder> (link_visuals.at(i)->geometry);
            radius=urdf_cylinder_ptr->radius;
            length=urdf_cylinder_ptr->length;

            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            createPtCloudFromCylinder(point_cloud,radius,length);
            link_point_cloud.push_back(point_cloud);
        }
        else if(link_visuals.at(i)->geometry->type == urdf::Geometry::SPHERE)
        {

            #ifdef CREATEPOINTCLOUDFROMVISUAL_LOG
              std::cout<<"------------------------------registering sphere------------------------------ " <<std::endl;
            #endif

            urdf::SphereSharedPtr urdf_sphere_ptr= urdf::static_pointer_cast <urdf::Sphere> (link_visuals.at(i)->geometry);
            radius=urdf_sphere_ptr->radius;
            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            createPtCloudFromSphere(point_cloud,radius);
            link_point_cloud.push_back(point_cloud);
        }
    }
}

void RobotModel::addGraspObject(urdf::CollisionSharedPtr grasp_object, std::string parent_link_name)
{
    //#define ADDGRASPOBJECT_LOG

    #ifdef ADDGRASPOBJECT_LOG
        std::cout<<" ==Start ADD grasp object =============================="<<std::endl;
    #endif

    // grasp object should have unique name.
    if (urdf_model_->links_.find(grasp_object->name) != urdf_model_->links_.end())
    {
        std::cout<<"Please give the grasp object a unique name. Grasp object with name "<<grasp_object->name<<" is already available"<<std::endl;
        return;
    }

    // creating a urdf link for the grasp object
    urdf::LinkSharedPtr grasp_link;
    grasp_link.reset(new urdf::Link);
    grasp_link->name = grasp_object->name;
    grasp_link->collision = grasp_object;

    // add the grasp object to urdf model
    urdf_model_->links_.insert(make_pair(grasp_link->name, grasp_link));

    // set the parent and child for the grasp link
    urdf::LinkSharedPtr parent_link;
    urdf_model_->getLink(parent_link_name, parent_link);

    //set parent link for grasp link
    grasp_link->setParent(parent_link);

    //set parent joint for grasp link
    urdf::JointSharedPtr grasp_object_joint;
    grasp_object_joint.reset(new urdf::Joint);

    grasp_object_joint->name = "grasp_object_joint" ;
    grasp_object_joint->child_link_name = grasp_object->name ;
    grasp_object_joint->parent_link_name = parent_link_name;
    grasp_object_joint->type = urdf::Joint::FIXED;
    /*grasp_object_joint->axis.x=0;grasp_object_joint->axis.y=0;grasp_object_joint->axis.z=1;
    grasp_object_joint->parent_to_joint_origin_transform.position.x=0;
    grasp_object_joint->parent_to_joint_origin_transform.position.y=0;
    grasp_object_joint->parent_to_joint_origin_transform.position.z=0;
    grasp_object_joint->parent_to_joint_origin_transform.rotation=urdf::Rotation(0,0,0,1);*/
    parent_link->child_links.push_back(grasp_link);
    grasp_link->parent_joint =grasp_object_joint;

    #ifdef ADDGRASPOBJECT_LOG
    std::cout<<"The following links are now available in the urdf_model"<<std::endl;
    for(std::map<std::string, urdf::LinkSharedPtr >::iterator it=urdf_model_->links_.begin(); it!=urdf_model_->links_.end();it++)
        std::cout<<it->second->name<<std::endl;
    #endif

    // creating a robot link for the given grasp object
    RobotLink robot_link;
    robot_link.setLinkName(grasp_object->name);
    robot_state_.robot_links_[grasp_object->name] = robot_link;

    robot_state_.robot_links_[grasp_object->name].setLinkCollision(grasp_object );
    robot_state_.robot_links_[grasp_object->name].setLinkDFSVisited(false);

    // setting frame for the grasp object
    KDL::Frame parent_link_in_base_link = robot_state_.robot_links_[parent_link_name].getLinkFrame();
    KDL::Frame grasp_object_frame_in_parent_frame;
    KDL::Frame grasp_object_frame_in_base_link;

    grasp_object_frame_in_parent_frame.p.data[0] = grasp_object->origin.position.x;
    grasp_object_frame_in_parent_frame.p.data[1] = grasp_object->origin.position.y;
    grasp_object_frame_in_parent_frame.p.data[2] = grasp_object->origin.position.z;

    grasp_object_frame_in_parent_frame.M = KDL::Rotation::Quaternion(grasp_object->origin.rotation.x, grasp_object->origin.rotation.y,
                                            grasp_object->origin.rotation.z, grasp_object->origin.rotation.w);

    grasp_object_frame_in_base_link = parent_link_in_base_link * grasp_object_frame_in_parent_frame;


    robot_state_.robot_links_[grasp_object->name].setLinkFrame(grasp_object_frame_in_base_link);

    robot_state_.robot_links_[grasp_object->name].calculateLinkVisualsPoseInGlobalPose();

    // disable the collision between the grasp object and its parent link
    srdf::Model::DisabledCollision disable_collision_grasp_object_w_parent_link;
    disable_collision_grasp_object_w_parent_link.link1_ = grasp_object->name;
    disable_collision_grasp_object_w_parent_link.link2_ = parent_link_name;
    disable_collision_grasp_object_w_parent_link.reason_ = "Adjacent";

    robot_collision_detector_->AbstractCollisionDetection::addDisabledCollisionPairs(disable_collision_grasp_object_w_parent_link);

    // register the grasp object to the collision manager.
    double grasp_rot_x = 0.0, grasp_rot_y = 0.0, grasp_rot_z = 0.0, grasp_rot_w = 0.0;    
    grasp_object_frame_in_base_link.M.GetQuaternion(grasp_rot_x, grasp_rot_y, grasp_rot_z, grasp_rot_w);
    
    /*fcl::Quaternion3f collision_quaternion_orientation (grasp_rot_x, grasp_rot_y, grasp_rot_z, grasp_rot_w);
    fcl::Vec3f collision_object_translation (grasp_object_frame_in_base_link.p.data[0],
                                             grasp_object_frame_in_base_link.p.data[1],
                                             grasp_object_frame_in_base_link.p.data[2]);
    */
    
    base::Pose collision_pose;
    collision_pose.position.x() = grasp_object_frame_in_base_link.p.data[0];
    collision_pose.position.y() = grasp_object_frame_in_base_link.p.data[1];
    collision_pose.position.z() = grasp_object_frame_in_base_link.p.data[2];
    
    collision_pose.orientation.x() = grasp_rot_x;
    collision_pose.orientation.y() = grasp_rot_y;
    collision_pose.orientation.z() = grasp_rot_z;
    collision_pose.orientation.w() = grasp_rot_w;


    // All collision object will have a suffix depending on the link collision.
    // i.e.,:   Consider a case in which a robot link_1's collision is decribed by 3 pritimive objects.
    //          The robot model stores the link collision as link_1_1, link_1_2, link_1_3. Thats how robot model
    //          is updated. Because of this issue, we need to add a string "_0" to the grasp object
    std::string grasp_object_name;
    grasp_object_name = grasp_object->name+"_0";


    if(grasp_object->geometry->type == urdf::Geometry::MESH)
    {
        #ifdef ADDGRASPOBJECT_LOG
            std::cout<<"------------------------------registering mesh file------------------------------ " <<std::endl;
        #endif
        urdf::MeshSharedPtr urdf_mesh_ptr= urdf::static_pointer_cast <urdf::Mesh> (grasp_object->geometry);

        //double scale_for_mesha_files_x=urdf_mesh_ptr->scale.x;
        //double scale_for_mesha_files_y=urdf_mesh_ptr->scale.y;
        //double scale_for_mesha_files_z=urdf_mesh_ptr->scale.z;
	
	Eigen::Vector3d mesh_scale(urdf_mesh_ptr->scale.x, urdf_mesh_ptr->scale.y, urdf_mesh_ptr->scale.z);

        boost::filesystem::path relative_path_of_mesh_file_in_urdf_file=urdf_mesh_ptr->filename;

        std::string urdf_directory_path=urdf_file_abs_path_.substr(0, urdf_file_abs_path_.find_last_of("/") );

        boost::filesystem::path abs_path_of_mesh_file =resolve_path( relative_path_of_mesh_file_in_urdf_file, urdf_directory_path );

        std::string abs_path_to_mesh_file = abs_path_of_mesh_file.string();
        robot_collision_detector_->registerMeshToCollisionManager(abs_path_to_mesh_file,mesh_scale, grasp_object_name, collision_pose, link_padding_);
    }
    else if(grasp_object->geometry->type == urdf::Geometry::BOX)
    {

        #ifdef ADDGRASPOBJECT_LOG
            std::cout<<"------------------------------registering box------------------------------ " <<std::endl;
        #endif

        urdf::BoxSharedPtr urdf_box_ptr= urdf::static_pointer_cast <urdf::Box> (grasp_object->geometry);

        double x = urdf_box_ptr->dim.x;
        double y = urdf_box_ptr->dim.y;
        double z = urdf_box_ptr->dim.z;

        robot_collision_detector_->registerBoxToCollisionManager( x,y,z, grasp_object_name, collision_pose, link_padding_);
    }
    else if(grasp_object->geometry->type == urdf::Geometry::CYLINDER)
    {

        #ifdef ADDGRASPOBJECT_LOG
            std::cout<<"------------------------------registering cylinder------------------------------" <<std::endl;
        #endif

        urdf::CylinderSharedPtr urdf_cylinder_ptr= urdf::static_pointer_cast <urdf::Cylinder> (grasp_object->geometry);
        double radius = urdf_cylinder_ptr->radius;
        double length = urdf_cylinder_ptr->length;
        robot_collision_detector_->registerCylinderToCollisionManager(radius, length, grasp_object_name,
                                                                                collision_pose,link_padding_);
    }
    else if(grasp_object->geometry->type == urdf::Geometry::SPHERE)
    {
        #ifdef ADDGRASPOBJECT_LOG
            std::cout<<"------------------------------registering sphere------------------------------ " <<std::endl;
        #endif

        urdf::SphereSharedPtr urdf_sphere_ptr= urdf::static_pointer_cast <urdf::Sphere> (grasp_object->geometry);
        double radius = urdf_sphere_ptr->radius;
        robot_collision_detector_->registerSphereToCollisionManager(radius,grasp_object_name, collision_pose,link_padding_  );
    }

    #ifdef ADDGRASPOBJECT_LOG
    std::cout<<""<<std::endl;
    std::cout<<"The following links are now available in the robot_state"<<std::endl;
    for(std::map<std::string, RobotLink>::iterator it=robot_state_.robot_links.begin();it!=robot_state_.robot_links.end();it++  )
        std::cout<<it->first<<std::endl;
    std::cout<<" ==============================End ADD grasp object=="<<std::endl;
    #endif


}


void RobotModel::removeGraspObject(const std::string grasp_object_name)
{
    //#define REMOVEGRASPOBJECT_LOG

    #ifdef REMOVEGRASPOBJECT_LOG
        std::cout<<" ==Start REMOVE grasp object =============================="<<std::endl;
    #endif

    #ifdef REMOVEGRASPOBJECT_LOG
    std::cout<<"The following links are available in the urdf_model before removing the link "<<grasp_object_name<<std::endl;
    for(std::map<std::string, urdf::LinkSharedPtr >::iterator it=urdf_model_->links_.begin(); it!=urdf_model_->links_.end();it++)
    {
        std::cout<<it->second->name<<std::endl;
    }
    std::cout<<""<<std::endl;
    #endif


    // remove grasp link from urdf model
    urdf::LinkConstSharedPtr link_ptr;
    if (urdf_model_->links_.find(grasp_object_name) == urdf_model_->links_.end())
    {
        std::cout<<" No grasp object is available with name "<< grasp_object_name<<std::endl;
        return;
    }
    else
    {
        // get the pointer of the grasp link in the urdf model
        link_ptr = urdf_model_->links_.find(grasp_object_name)->second;

        //detach or remove from its parent link
        urdf::LinkSharedPtr parent_link;
        parent_link = link_ptr->getParent();
        _removeLinkName = grasp_object_name;
        parent_link->child_links.erase(std::remove_if(parent_link->child_links.begin(), parent_link->child_links.end(), isLinkListed),parent_link->child_links.end() );

        // now remove the grasp link from the urdf model
        urdf_model_->links_.erase(grasp_object_name);
    }

    #ifdef REMOVEGRASPOBJECT_LOG
    std::cout<<"The following links are available in the urdf_model after removing the link "<<grasp_object_name<<std::endl;
    for(std::map<std::string, urdf::LinkSharedPtr >::iterator it=urdf_model_->links_.begin(); it!=urdf_model_->links_.end();it++)
    {
        std::cout<<it->second->name<<std::endl;
    }
    #endif
    // remove grasp link from robot model
    robot_state_.robot_links_.erase(grasp_object_name);

    // remove the grasp link form collision pair
    //(this->self_collision_detection.removeDisabledCollisionLink(grasp_object_name);
    robot_collision_detector_->removeDisabledCollisionLink(grasp_object_name);

     // remove the grasp link from collision data base
    //this->self_collision_detection.removeSelfCollisionObject(grasp_object_name);    
    robot_collision_detector_->removeSelfCollisionObject(grasp_object_name);

    #ifdef REMOVEGRASPOBJECT_LOG
        std::cout<<" ==============================End REMOVE grasp =="<<std::endl;
    #endif

}

void RobotModel::removeWorldObject(const std::string world_object_name)
{
    //#define REMOVEWORLDOBJECT_LOG

    #ifdef REMOVEGRASPOBJECT_LOG
        std::cout<<" ==Start REMOVE world object =============================="<<std::endl;
    #endif

    world_collision_detector_->removeWorldCollisionObject(world_object_name);

    #ifdef REMOVEWORLDOBJECT_LOG
        std::cout<<" ==============================End REMOVE grasp =="<<std::endl;
    #endif
}

void RobotModel::updateJoint(std::string joint_name, double joint_value)
{

    if(this->urdf_model_->getJoint(joint_name) )
    {
        if(! robot_state_.robot_joints_[joint_name].isMimicJoint() )
        {

            std::string start_link_name= this->urdf_model_->getJoint(joint_name)->parent_link_name;

            if(robot_state_.robot_joints_[joint_name].mimic_joints_map_.size() > 0)
            {
                for (std::map< std::string, MimicJoint >::iterator it = robot_state_.robot_joints_[joint_name].mimic_joints_map_.begin();
                     it!= robot_state_.robot_joints_[joint_name].mimic_joints_map_.end(); ++it)
                {
                    robot_state_.robot_joints_[it->first].setJointValue ( (joint_value * it->second.multiplier) + it->second.offset);
                }
            }
            else
                robot_state_.robot_joints_[joint_name].setJointValue (joint_value);

            std::vector<std:: string> visited_links;
     
            dfsTraversing(start_link_name, visited_links);
 
            settingVisitedFlagLinkToFalse(visited_links);
	    //std::cout<<"list of visited link for joint "<<joint_name.c_str()<<"  "<<visited_links.size()<<std::endl;
            LOG_DEBUG_S<<"========================== list of visited link for joint "<<joint_name.c_str();

            /*for(std::size_t i=0;i<visited_links.size();i++)
            {
                LOG_DEBUG_S<<visited_links.at(i).c_str();
            }*/

            std::string name_of_visited_link, name_of_collision_object;
            //double quaternion_w,quaternion_x,quaternion_y,quaternion_z, translation_x,translation_y,translation_z;
	    base::Pose collision_object_pose;


            ////////////////////link visual and collision pose has been updated in dfstraverse, but we have to update teh collision manger //////////////////////////////////
            for(std::size_t i=0;i<visited_links.size();i++)
            {
                name_of_visited_link=visited_links.at(i);


                std::vector<urdf::CollisionSharedPtr >link_collisions = robot_state_.robot_links_[name_of_visited_link].getLinkCollisions() ;

                for(std::size_t j=0;j<link_collisions.size();j++ )
                {
                    urdf::CollisionSharedPtr link_collision_global_pose =link_collisions.at(j);

		    collision_object_pose.position.x() = link_collision_global_pose->origin.position.x;
                    collision_object_pose.position.y() = link_collision_global_pose->origin.position.y;
                    collision_object_pose.position.z() = link_collision_global_pose->origin.position.z;
		    	    
		    collision_object_pose.orientation.w() = link_collision_global_pose->origin.rotation.w;
                    collision_object_pose.orientation.x() = link_collision_global_pose->origin.rotation.x;
                    collision_object_pose.orientation.y() = link_collision_global_pose->origin.rotation.y;
                    collision_object_pose.orientation.z() = link_collision_global_pose->origin.rotation.z;


                    name_of_collision_object=name_of_visited_link+"_"+lexical_cast<std::string>(j);
                    robot_collision_detector_->updateCollisionObjectTransform(name_of_collision_object, collision_object_pose);
                }

            }
            
        }
    }
}

void RobotModel::updateJointGroup(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values)
{
    assert(joint_names.size() == joint_values.size());	    
    //auto start_time = std::chrono::high_resolution_clock::now(); 
    
    for(std::size_t i=0;i<joint_names.size();i++)
        this->updateJoint(joint_names.at(i) ,joint_values(i) ) ;
      // 	    auto finish_time = std::chrono::high_resolution_clock::now();
   //std::chrono::duration<double> elapsed = finish_time - start_time;
   //std::cout << "Elapsed time colli: " << elapsed.count() << " s\n";
}

void RobotModel::updateJointGroup(const base::samples::Joints &joint_values)
{

    for(std::size_t i = 0; i < joint_values.size(); i++)
    {	
        updateJoint(joint_values.names[i], joint_values.elements[i].position) ;
    }
}

void RobotModel::updateJointGroup(const std::map< std::string ,double > &joint_values)
{

    for(std::map<std::string,double>::const_iterator it=joint_values.begin();it!=joint_values.end();++it)
    {
        this->updateJoint(it->first, it->second) ;
    }
}

void RobotModel::updateJointGroup( const std::vector<std::string> &joint_names, const std::vector<double> &joint_values)
{
    std::string joint_name;
    for(std::size_t i=0;i<joint_names.size();i++)
    {
        joint_name=joint_names.at(i);
        this->updateJoint(joint_name ,joint_values.at(i) ) ;
    }
}

bool RobotModel::isStateValid(int self_collision_num_max_contacts, int external_collision_manager_num_max_contacts)
{
    if (robot_collision_detector_->checkSelfCollision(self_collision_num_max_contacts))
    {
	
	LOG_DEBUG("[RobotModel]: There is no self collision, now checking for collision against environment");        
	
	if(robot_collision_detector_->checkWorldCollision(external_collision_manager_num_max_contacts))
	{
            LOG_DEBUG("[RobotModel]: There is no collision against environment" );           
            return true;
        }
        else
	{           
            LOG_DEBUG("[RobotModel]: There is collision against environment" );
            return false;
        }
    }    
    return false;
}

/*
bool RobotModel::IsStateIsValid(int self_collision_num_max_contacts, int external_collision_manager_num_max_contacts)
{
//    #define ISSTATEISVALID_LOG
    bool is_state_of_robot_and_evironment_is_valid;

    #ifdef ISSTATEISVALID_LOG
    std::cout<<"checking for robot self collision" <<std::endl;
    #endif
    if (this->self_collision_detection.IsStateIsValid(self_collision_num_max_contacts))
    {
        #ifdef ISSTATEISVALID_LOG
        std::cout<<"there is no self collision, checking for collision against environment" <<std::endl;
        #endif

        is_state_of_robot_and_evironment_is_valid=this->self_collision_detection.checkCollisionAgainstExternalCollisionManager( collision_detector_.getCollisionManager(),external_collision_manager_num_max_contacts );
        if (is_state_of_robot_and_evironment_is_valid)
        {
            #ifdef ISSTATEISVALID_LOG
            std::cout<<"there is no collision against environment" <<std::endl;
            #endif
            return true;
        }
        else
        {
            #ifdef ISSTATEISVALID_LOG
            std::cout<<"there is collision against environment" <<std::endl;
            #endif
            return false;
        }

    }
    else
    {
        return false;
    }
}

bool RobotModel::DistanceOfClosestObstacleToRobot(DistanceData  &distance_data)
{
    bool IsStateIsValid=this->self_collision_detection.DistanceOfClosestObstacleToRobot(.getCollisionManager(),distance_data);
    return IsStateIsValid;
}
*/

/*void RobotModel::ConvertPoseBetweenFrames( const std::string B_Frame_Name, const base::samples::RigidBodyState &F_B_C , const std::string &A_Frame_Name ,
					   base::samples::RigidBodyState &F_A_C )
{
    KDL::Frame kdl_frame_f_b_c;
    kdl_frame_f_b_c.p.data[0] = F_B_C.position(0);
    kdl_frame_f_b_c.p.data[1] = F_B_C.position(1);
    kdl_frame_f_b_c.p.data[2] = F_B_C.position(2);

    kdl_frame_f_b_c.M = KDL::Rotation::Quaternion(F_B_C.orientation.x(), F_B_C.orientation.y(),
    F_B_C.orientation.z(), F_B_C.orientation.w() );
    
    KDL::Frame kdl_frame_f_a_c; 
    ConvertPoseBetweenFrames(B_Frame_Name, kdl_frame_f_b_c , A_Frame_Name, kdl_frame_f_a_c );
    
    F_A_C.position(0) = kdl_frame_f_a_c.p.data[0];
    F_A_C.position(1) = kdl_frame_f_a_c.p.data[1];
    F_A_C.position(2) = kdl_frame_f_a_c.p.data[2];
    
    kdl_frame_f_a_c.M.GetQuaternion(F_A_C.orientation.x(),F_A_C.orientation.y(),F_A_C.orientation.z(),F_A_C.orientation.w());
}*/

void RobotModel::ConvertPoseBetweenFrames( const std::string B_Frame_Name, const KDL::Frame &F_B_C , const std::string &A_Frame_Name ,KDL::Frame &F_A_C )
{

/*
    F_A_C = F_A_B * F_B_C

    F_A_C ==> will be calculated by this function
    F_A_B ==> calculated by kdl FK, A is chain_root, B is chain_tip
    F_B_C ==> is given

    A ==> is chain_root
    B ==> is chain_tip
    C ==> given pose is in this frame

*/
//        You can use the operator * to compose frames. If you have a Frame F_A_B that expresses the pose of frame B wrt frame A,
//        and a Frame F_B_C that expresses the pose of frame C wrt to frame B, the calculation of Frame F_A_C that
//        expresses the pose of frame C wrt to frame A is as follows:
//        Frame F_A_C = F_A_B * F_B_C;




//    #define CONVERTPOSEBETWEENFRAMES

    KDL::Frame F_A_B;
    std::string chain_root_link, chain_tip_link;
    chain_root_link= A_Frame_Name;
    chain_tip_link=B_Frame_Name;
    KDL::JntArray kdl_chain_joint_array;

    kdl_tree_.getChain(chain_root_link , chain_tip_link , kdl_chain_);
    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
    kdl_chain_joint_array.resize(kdl_chain_.getNrOfJoints());
    std::string link_name;
    std::string joint_name;
    int joint_type;
    int j=0;
    double joint_value;
    for(std::size_t i=0;i<kdl_chain_.segments.size();i++)
    {
        link_name=kdl_chain_.getSegment(i).getName();
        joint_name=kdl_chain_.getSegment(i).getJoint().getName();
        joint_type=this->urdf_model_->getJoint(joint_name)->type;
        #ifdef CONVERTPOSEBETWEENFRAMES
        std::string joint_array_names[]={"UNKNOWN", "REVOLUTE", "CONTINUOUS", "PRISMATIC", "FLOATING", "PLANAR", "FIXED"};
        std::cout<<"the link name is " <<link_name<<std::endl;
        std::cout<<"the joint is:" << joint_name <<std::endl;
        std::cout<<"joint type is "<<joint_array_names[joint_type] <<std::endl;
        #endif
        if(joint_type!=urdf::Joint::FIXED)
        {
            joint_value=robot_state_.robot_joints_[joint_name].getJointValue();
            kdl_chain_joint_array.data[j]=joint_value;
            j++;
            #ifdef CONVERTPOSEBETWEENFRAMES
            std::cout<<"joint_value: " <<joint_value <<std::endl;
            std::cout<<"---------------------------------------------------------------" <<std::endl;
            #endif
        }
    }
    fk_solver.JntToCart(kdl_chain_joint_array, F_A_B);
    F_A_C=F_A_B*F_B_C;

}

void RobotModel::getRobotCollisions(std::vector<urdf::CollisionSharedPtr > &  robotCollisions)
{
    // this will return the Collisons (defined in the urdf) of robot in global pose
    std::vector<urdf::CollisionSharedPtr > link_collisions;
    for(std::map<std::string, RobotLink> ::iterator it=robot_state_.robot_links_.begin();it!=robot_state_.robot_links_.end();it++)
    {
        link_collisions=it->second.getLinkCollisions();
        for(std::size_t i=0;i<link_collisions.size();i++)
        {
           robotCollisions.push_back( link_collisions.at(i) );
        }
    }
}

void RobotModel::getRobotVisuals(std::vector<urdf::VisualSharedPtr > &  robotVisuals)
{
    // this will return the Collisons (defined in the urdf) of robot in global pose
    std::vector<urdf::VisualSharedPtr > link_Visuals;
    for(std::map<std::string, RobotLink> ::iterator it=robot_state_.robot_links_.begin();it!=robot_state_.robot_links_.end();it++)
    {
        link_Visuals=it->second.getLinkVisuals();
        for(std::size_t i=0;i<link_Visuals.size();i++)
        {
           robotVisuals.push_back( link_Visuals.at(i) );
        }
    }
}

/*
void RobotModel::addCollisionsToWorld(boost::shared_ptr<fcl::CollisionObject> & collisionObject_ptr, std::string link_name)
{
    robot_collision_detector_.getCollisionManager()->clear();
    this->world_collision_detection.getCollisionManager()->registerObject(collisionObject_ptr.get());
}*/

void RobotModel::addCollisionsToWorld(urdf::CollisionSharedPtr &  robotCollision, std::string link_name, std::string collision_object_name)
{

    int numberOfObjectsInCollisionManger=   world_collision_detector_->numberOfObjectsInCollisionManger();
    double radius, length ,x,y,z; //, scale_for_mesha_files_x, scale_for_mesha_files_y , scale_for_mesha_files_z ;



    std::string abs_path_to_mesh_file;


    boost::filesystem::path relative_path_of_mesh_file_in_urdf_file;
    boost::filesystem::path urdf_abs_path;
    boost::filesystem::path abs_path_of_mesh_file;
    std::string urdf_directory_path;


    if(collision_object_name.empty())
    {
        collision_object_name=link_name+"_" +lexical_cast<std::string>(numberOfObjectsInCollisionManger);
    }


    #ifdef ADDCOLLISIONSTOWORLD_LOG
    std::cout<<"collision_object_name is  "<< collision_object_name <<std::endl;
    #endif

    //fcl::Quaternion3f collision_quaternion_orientation (robotCollision->origin.rotation.w,robotCollision->origin.rotation.x, robotCollision->origin.rotation.y,robotCollision->origin.rotation.z);
    //fcl::Vec3f collision_object_translation (robotCollision->origin.position.x,robotCollision->origin.position.y,robotCollision->origin.position.z);
    
    base::Pose collision_object_pose;
    collision_object_pose.position.x() = robotCollision->origin.position.x;
    collision_object_pose.position.y() = robotCollision->origin.position.y;
    collision_object_pose.position.z() = robotCollision->origin.position.z;
    collision_object_pose.orientation.x() = robotCollision->origin.rotation.x;
    collision_object_pose.orientation.y() = robotCollision->origin.rotation.y;
    collision_object_pose.orientation.z() = robotCollision->origin.rotation.z;
    collision_object_pose.orientation.w() = robotCollision->origin.rotation.w;

    if(robotCollision->geometry->type == urdf::Geometry::MESH)
    {
        #ifdef ADDCOLLISIONSTOWORLD_LOG
            std::cout<<"------------------------------registering mesh file------------------------------ " <<std::endl;
        #endif
        urdf::MeshSharedPtr urdf_mesh_ptr= urdf::static_pointer_cast <urdf::Mesh> (robotCollision->geometry);
        //scale_for_mesha_files_x=urdf_mesh_ptr->scale.x;
        //scale_for_mesha_files_y=urdf_mesh_ptr->scale.y;
        //scale_for_mesha_files_z=urdf_mesh_ptr->scale.z;
	
	Eigen::Vector3d mesh_scale(urdf_mesh_ptr->scale.x, urdf_mesh_ptr->scale.y, urdf_mesh_ptr->scale.z);

        relative_path_of_mesh_file_in_urdf_file=urdf_mesh_ptr->filename;
        urdf_abs_path = urdf_file_abs_path_;



        urdf_directory_path = urdf_file_abs_path_.substr(0, urdf_file_abs_path_.find_last_of("/") );
	

        abs_path_of_mesh_file =resolve_path( relative_path_of_mesh_file_in_urdf_file, urdf_directory_path );

        abs_path_to_mesh_file=abs_path_of_mesh_file.string();
        world_collision_detector_->registerMeshToCollisionManager(abs_path_to_mesh_file, mesh_scale, collision_object_name, collision_object_pose, link_padding_);
    }
    else if(robotCollision->geometry->type == urdf::Geometry::BOX)
    {

        #ifdef ADDCOLLISIONSTOWORLD_LOG
            std::cout<<"------------------------------registering box------------------------------ " <<std::endl;
        #endif

        urdf::BoxSharedPtr urdf_box_ptr= urdf::static_pointer_cast <urdf::Box> (robotCollision->geometry);
        x=urdf_box_ptr->dim.x;
        y=urdf_box_ptr->dim.y;
        z=urdf_box_ptr->dim.z;
        world_collision_detector_->registerBoxToCollisionManager( x,y,z,collision_object_name, collision_object_pose, link_padding_);

    }
    else if(robotCollision->geometry->type == urdf::Geometry::CYLINDER)
    {

        #ifdef ADDCOLLISIONSTOWORLD_LOG
            std::cout<<"------------------------------registering cylinder------------------------------" <<std::endl;
        #endif

        urdf::CylinderSharedPtr urdf_cylinder_ptr= urdf::static_pointer_cast <urdf::Cylinder> (robotCollision->geometry);
        radius=urdf_cylinder_ptr->radius;
        length=urdf_cylinder_ptr->length;
        world_collision_detector_->registerCylinderToCollisionManager(radius , length, collision_object_name,collision_object_pose,link_padding_);
    }
    else if(robotCollision->geometry->type == urdf::Geometry::SPHERE)
    {

        #ifdef ADDCOLLISIONSTOWORLD_LOG
            std::cout<<"------------------------------registering sphere------------------------------ " <<std::endl;
        #endif

        urdf::SphereSharedPtr urdf_sphere_ptr= urdf::static_pointer_cast <urdf::Sphere> (robotCollision->geometry);
        radius=urdf_sphere_ptr->radius;
        world_collision_detector_->registerSphereToCollisionManager(radius,collision_object_name, collision_object_pose ,link_padding_  );
    }
}

void RobotModel::generateRandomJointValue(const std::string  &planningGroupName,std::map<std::string, double  >   &planning_groups_joints_with_random_values)
{
    std::vector< std::pair<std::string,urdf::Joint >  >   planning_groups_joints;

    std::string base_link, tip_link;
    this->getPlanningGroupJointinformation(planningGroupName , planning_groups_joints, base_link,  tip_link);
    for(std::vector< std::pair<std::string,urdf::Joint >  >::iterator it=planning_groups_joints.begin();it!=planning_groups_joints.end();it++ )
    {
        double random_joint_value= randomFloat(it->second.limits->lower,it->second.limits->upper);
        planning_groups_joints_with_random_values[it->first]=random_joint_value;
    }
    return;
}

float RobotModel::randomFloat(const float& min,const  float &max)
{
    srand(time(NULL));
    float r = (float)rand() / (float)RAND_MAX;
    return min + r * (max - min);
}

/*These function are not used

std::string RobotModel::getURDFFileAbsolutePath()
{
    return urdf_file_abs_path_;
}

WorldCollision &RobotModel::getWorlCollision()
{
    return this->world_collision_detection;
}


std::vector<fcl::Contact> &RobotModel::getContactOfSelfCollision()
{
    return this->self_collision_detection.getSelfContacts();
}

std::vector<fcl::Contact> &RobotModel::getContactsAgainstExternalCollisionManager()
{
    //return this->world_collision_detection.getContactsAgainstExternalCollisionManager();
    return this->self_collision_detection.getContactsAgainstExternalCollisionManager();
}

// true means no collision
bool RobotModel::CheckRobotEnvironmentCollision()
{
    bool robot_env_is_collision_free=this->self_collision_detection.checkCollisionAgainstExternalCollisionManager( this->world_collision_detection.getCollisionManager() );
    return robot_env_is_collision_free;
}

// true means no collision
bool RobotModel::CheckSelfCollision(int num_max_contacts)
{
    bool robot_without_self_collision=this->self_collision_detection.IsStateIsValid(num_max_contacts);
    return robot_without_self_collision;
}



std::vector<fcl::CollisionObject*> RobotModel::getObjectIncollisionAgainstExternalCollisionManager()
{
    return this->self_collision_detection.getObjectIncollisionAgainstExternalCollisionManager();
}



std::vector < std::pair<fcl::CollisionObject*,fcl::CollisionObject* > > &RobotModel::getSelfCollisionObject()
{
    return this->self_collision_detection.getSelfCollisionObject();
}


std::vector<collision_detection::DistanceInformation>& RobotModel::getSelfDistanceInfo()
{
    collision_detector_->computeSelfDistanceInfo();
    return collision_detector_->getSelfDistanceInfo();
}
*/

void RobotModel::selfFilterUsingVisual(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene_ptr, std::string sensor_frame_name)
{
//    #define SELFFILTERUSINGVISUAL_LOG
    #ifdef SELFFILTERUSINGVISUAL_LOG
    double start_transforming_poit_clouds = omp_get_wtime();
    #endif

    RobotState robot_state= this->getRobotState();
    std::string A_Frame_Name=sensor_frame_name;
    std::string B_Frame_Name=this->getURDF()->getRoot()->name ;


    pcl::PointCloud<pcl::PointXYZ> link_visual_pointcloud_transformed_in_sensor_frame, robot_visuals_pointcloud;

    for(std::map<std::string, RobotLink>::iterator it=robot_state.robot_links_.begin();it!=robot_state.robot_links_.end();it++)
    {
        KDL::Frame link_visual_pose_in_sensor_frame;
        Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix;
        std::vector<urdf::VisualSharedPtr > link_visuals=it->second.getLinkVisuals();
        std::vector<pcl::PointCloud<pcl::PointXYZ> > visual_point_cloud=it->second.getVisualPointCloud();
        for(std::size_t i=0;i<link_visuals.size();i++)
        {
            KDL::Frame link_visual_pose_in_base_link=toKdl( link_visuals.at(i)->origin);
            this->ConvertPoseBetweenFrames( B_Frame_Name, link_visual_pose_in_base_link , A_Frame_Name , link_visual_pose_in_sensor_frame);
            KDLFrameToEigenMatrix(link_visual_pose_in_sensor_frame, link_visual_pose_in_sensor_frame_eigen_matrix);
            pcl::transformPointCloud (visual_point_cloud.at(i), link_visual_pointcloud_transformed_in_sensor_frame, link_visual_pose_in_sensor_frame_eigen_matrix);
            robot_visuals_pointcloud=robot_visuals_pointcloud+link_visual_pointcloud_transformed_in_sensor_frame;
        }
    }
//    pcl::io::savePCDFile("robot.pcd",robot_visuals_pointcloud);


    #ifdef SELFFILTERUSINGVISUAL_LOG
    double end_transforming_poit_clouds = omp_get_wtime();

    std::cout<<"transforming_poit_clouds took "<< end_transforming_poit_clouds-start_transforming_poit_clouds<<std::endl;

    double start_subtracting_pointclouds = omp_get_wtime();
    #endif

    subtractingPtClouds( new_scene_ptr,scene_ptr , boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(robot_visuals_pointcloud )    );

    #ifdef SELFFILTERUSINGVISUAL_LOG
    double end_subtracting_pointclouds = omp_get_wtime();
    std::cout<<"subtracting_pointclouds took "<< end_subtracting_pointclouds-start_subtracting_pointclouds<<std::endl;
    #endif
    return;
}



void RobotModel::selfFilterUsingCollision(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene_ptr, std::string sensor_frame_name)
{
//    #define SELFFILTERUSINGCOLLISION_LOG
    #ifdef SELFFILTERUSINGCOLLISION_LOG
    double start_transforming_poit_clouds = omp_get_wtime();
    #endif

    RobotState robot_state = getRobotState();
    std::string A_Frame_Name=sensor_frame_name;
    std::string B_Frame_Name=this->getURDF()->getRoot()->name ;


    pcl::PointCloud<pcl::PointXYZ> link_collision_pointcloud_transformed_in_sensor_frame, robot_collisions_pointcloud;

    for(std::map<std::string, RobotLink>::iterator it=robot_state.robot_links_.begin();it!=robot_state.robot_links_.end();it++)
    {
        KDL::Frame link_collision_pose_in_sensor_frame;
        Eigen::Affine3f link_collision_pose_in_sensor_frame_eigen_matrix;
        std::vector<urdf::CollisionSharedPtr > link_collisions=it->second.getLinkCollisions();
        std::vector<pcl::PointCloud<pcl::PointXYZ> > collision_point_cloud=it->second.getCollisionPointCloud();
        for(std::size_t i=0;i<link_collisions.size();i++)
        {
            KDL::Frame link_collision_pose_in_base_link=toKdl( link_collisions.at(i)->origin);
            this->ConvertPoseBetweenFrames( B_Frame_Name, link_collision_pose_in_base_link , A_Frame_Name , link_collision_pose_in_sensor_frame);
            KDLFrameToEigenMatrix(link_collision_pose_in_sensor_frame, link_collision_pose_in_sensor_frame_eigen_matrix);
            pcl::transformPointCloud (collision_point_cloud.at(i), link_collision_pointcloud_transformed_in_sensor_frame, link_collision_pose_in_sensor_frame_eigen_matrix);
            robot_collisions_pointcloud=robot_collisions_pointcloud+link_collision_pointcloud_transformed_in_sensor_frame;
        }
    }

//    pcl::io::savePCDFile("robot_Collision.pcd",robot_collisions_pointcloud);


    #ifdef SELFFILTERUSINGCOLLISION_LOG
    double end_transforming_poit_clouds = omp_get_wtime();

	std::cout<<"transforming_poit_clouds took "<< end_transforming_poit_clouds-start_transforming_poit_clouds<<std::endl;

    double start_subtracting_pointclouds = omp_get_wtime();
    #endif

    subtractingPtClouds( new_scene_ptr,scene_ptr , boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(robot_collisions_pointcloud )    );

    #ifdef SELFFILTERUSINGCOLLISION_LOG
    double end_subtracting_pointclouds = omp_get_wtime();

	std::cout<<"subtracting_pointclouds took "<< end_subtracting_pointclouds-start_subtracting_pointclouds<<std::endl;
    #endif

    return;
}

void RobotModel::pclStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (input_cloud);
	sor.setMeanK (50);
  	sor.setStddevMulThresh (1.0);
  	sor.filter (*filtered_cloud);
}

void RobotModel::printWorldCollisionObject()
{
     world_collision_detector_->printCollisionObject();

}



}// end namespace 
