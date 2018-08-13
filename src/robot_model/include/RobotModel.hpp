#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <string>
#include <vector>

#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <time.h>




#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/joint.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf_model/types.h>
#include <urdf_model/link.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "RobotLink.hpp"
#include "RobotJoint.hpp"

#include <base-logging/Logging.hpp>
#include <base/Pose.hpp>
#include <base/samples/Joints.hpp>

/*
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>*/

#include <collision_detection/abstract/AbstractCollisionDetection.hpp>
#include <collision_detection/abstract/MeshLoader.hpp>
#include <kinematics_library/KinematicsFactory.hpp>

#include <chrono>  // for high_resolution_clock - Available for C+11


namespace motion_planners
{

template<typename to, typename from>
to lexical_cast(from const &x)
{
    std::stringstream os;
    to ret;
    os << x;
    os >> ret;
    return ret;
}



class RobotState
{
    public:
        RobotState(){};
        std::map<std::string, RobotLink> robot_links_;
        std::map<std::string, RobotJoint> robot_joints_;
};

class RobotModel
{

    public:
	RobotModel(std::string urdf_file, std::string srdf_file, std::string planning_group_name,
		  double link_padding = 0.99);

        bool initialization();    

        bool getPlanningGroupJointinformation(  const std::string  planningGroupName,
                                                std::vector< std::pair<std::string,urdf::Joint> > &planning_groups_joints,
                                                std::string &base_link,  std::string &tip_link);

        void getPlanningGroupJointsName(const std::string planningGroupName,
                                        std::vector< std::string> &planning_group_joints_name);

        void setSRDF(boost::shared_ptr<srdf::Model> &srdf_model_);

        void setURDF(urdf::ModelInterfaceSharedPtr &urdf_model_);

        boost::shared_ptr<srdf::Model>  const & getSRDF();

        urdf::ModelInterfaceSharedPtr const &  getURDF();

        void dfsTraversing(std::string start_link_name, std::vector<std::string>& visited_links);

        void settingVisitedFlagLinkToFalse();

        void settingVisitedFlagLinkToFalse(std::vector<std::string> visted_links_names);
        
        RobotState  getRobotState();

        void setRobotState(RobotState &robot_state );

        void initializeLinksCollisions();

        void populate_disabled_collision_pairs();

        void updateJoint(std::string joint_name, double joint_value);
	
	void updateJointGroup(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values);

        void updateJointGroup(const base::samples::Joints &joint_values);
	  
	void updateJointGroup(const std::map< std::string ,double > &joint_values);

        void updateJointGroup(const std::vector<std::string> &joint_names,const std::vector<double> &joint_values) ;

        bool isStateValid(int self_collision_num_max_contacts=1, int external_collision_manager_num_max_contacts=1);
	
	//void ConvertPoseBetweenFrames( const std::string B_Frame_Name, const base::samples::RigidBodyState &F_B_C , const std::string &A_Frame_Name ,
	//				   base::samples::RigidBodyState &F_A_C );

        void ConvertPoseBetweenFrames( const std::string B_Frame_Name, const KDL::Frame &F_B_C , const std::string &A_Frame_Name ,KDL::Frame &F_A_C );

        void getRobotCollisions(std::vector<urdf::CollisionSharedPtr > &  robotCollisions);

        void getRobotVisuals(std::vector<urdf::VisualSharedPtr > &  robotVisuals);

        void addCollisionsToWorld(urdf::CollisionSharedPtr &  robotCollision, std::string link_name, std::string collision_object_name="");

        //S void addCollisionsToWorld(boost::shared_ptr<fcl::CollisionObject> & collisionObject_ptr, std::string link_name);

        void generateRandomJointValue(const std::string  &planningGroupName, std::map<std::string, double>   &planning_groups_joints_with_random_values);

        float randomFloat(const float& min,const  float &max);

        boost::filesystem::path resolve_path( const boost::filesystem::path& p, const boost::filesystem::path& base = boost::filesystem::current_path());

        std::string getURDFFileAbsolutePath();
	
	
	inline void setRobotCollisionDetector(collision_detection::AbstractCollisionPtr collision_detector){robot_collision_detector_ = collision_detector;}
	void setWorldCollisionDetector(collision_detection::AbstractCollisionPtr collision_detector);

	inline void setKinematicsSolver(kinematics_library::RobotKinematicsPtr robot_kinematics){robot_kinematics_ = robot_kinematics;}

/*        WorldCollision &getWorlCollision();


        template <class PointT>
        void addCollisionsToWorld(const pcl::PointCloud<PointT>& pclCloud, Eigen::Vector3d relative_position_to_link, Eigen::Quaterniond relative_rotation_to_link,
                                  std::string link_name, double octree_resolution, std::string collision_object_name="")
        {
            addCollisionsToWorld(pclCloud, relative_position_to_link.x(), relative_position_to_link.y(), relative_position_to_link.z(),
                                 relative_rotation_to_link.x(), relative_rotation_to_link.y(), relative_rotation_to_link.z(), relative_rotation_to_link.w(),
                                 link_name, octree_resolution, collision_object_name);


        }

        template <class PointT>
        void addCollisionsToWorld(const pcl::PointCloud<PointT>& pclCloud, double relative_pose_to_link_x, double relative_pose_to_link_y,
                                              double relative_pose_to_link_z,double relative_pose_to_link_quaternion_x,double relative_pose_to_link_quaternion_y ,double relative_pose_to_link_quaternion_z,double relative_pose_to_link_quaternion_w ,
                                              std::string link_name, double octree_resolution,std::string collision_object_name="")
        {

            fcl::Quaternion3f collision_quaternion_orientation (relative_pose_to_link_quaternion_w,relative_pose_to_link_quaternion_x, relative_pose_to_link_quaternion_y,relative_pose_to_link_quaternion_z);
            fcl::Vec3f collision_object_translation (relative_pose_to_link_x,relative_pose_to_link_y,relative_pose_to_link_z);

            if(collision_object_name.empty())
            {
                int numberOfObjectsInCollisionManger= world_collision_detection.numberOfObjectsInCollisionManger();
                collision_object_name=link_name+"_" +lexical_cast<std::string>(numberOfObjectsInCollisionManger);
            }
            this->world_collision_detection.registerPCLPointCloudToCollisionObjectManager(pclCloud,octree_resolution,collision_object_name, collision_quaternion_orientation,collision_object_translation );
            return;
        }

        bool distanceOfClosestObstacleToRobot(DistanceData  &distance_data);

        std::vector<fcl::Contact> &getContactOfSelfCollision();
        std::vector<fcl::Contact> &getContactsAgainstExternalCollisionManager();
*/
        /* true means no collision */
        bool checkRobotEnvironmentCollisin();
        /* true means no collision */
        bool checkSelfCollision(int num_max_contacts=1);

/*        std::vector<fcl::CollisionObject*> getObjectIncollisionAgainstExternalCollisionManager();
        std::string getIKFASTSharedObjectAbsolutePath()
        {
            return this->ik_fast_shared_object_abs_path;
        }

        std::vector < std::pair<fcl::CollisionObject*,fcl::CollisionObject* > > &getSelfCollisionObject();
*/
        void selfFilterUsingCollision(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene_ptr, std::string sensor_frame_name);

        void selfFilterUsingVisual(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene_ptr, std::string sensor_frame_name);

        std::vector< collision_detection::DistanceInformation>& getSelfDistanceInfo();

        //void createPointCloudFromBox(pcl::PointCloud<pcl::PointXYZ> &transformed_box_cloud,double x, double y, double z);

        
	//void createPointCloudFromCylinder(pcl::PointCloud<pcl::PointXYZ> &transformed_cylinder_cloud, double radius, double height, int number_of_step_alpha=20 );

        //void createPointCloudFromSphere(pcl::PointCloud<pcl::PointXYZ> &transformed_sphere_cloud, double radius, int number_of_step_alpha=20, int number_of_step_beta=20);

        void createPointCloudFromVisual(std::vector<urdf::VisualSharedPtr > &link_visuals, std::vector<pcl::PointCloud<pcl::PointXYZ> > &link_point_cloud );

        void createPointCloudFromCollision(std::vector<urdf::CollisionSharedPtr> &link_collisions, std::vector<pcl::PointCloud<pcl::PointXYZ> > &link_point_cloud );

        void computeJacobain(const std::string &chain_root_link,const  std::string& tip_link, std::map<std::string, double> joints_name_values, KDL::Jacobian  &jacobian);

        void manipulabilityIndex(KDL::Jacobian  &jacobian, double &manipulability_index);

        void createPtCloudFromBox(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_box_cloud_ptr,double x, double y, double z, Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix, bool dense);
	
	void createPtCloudFromBox(pcl::PointCloud<pcl::PointXYZ> &box_cloud,double x, double y, double z);

        void createPtCloudFromCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cylinder_cloud_ptr, double radius, double height, Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix, int number_of_step_alpha=10, bool dense=false );
	
	void createPtCloudFromCylinder(pcl::PointCloud<pcl::PointXYZ> &cylinder_cloud, double radius, double height, int number_of_step_alpha=20 );

        void createPtCloudFromSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_sphere_cloud_ptr, double radius, Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix, int number_of_step_alpha=10, int number_of_step_beta=10);
	
	void createPtCloudFromSphere(pcl::PointCloud<pcl::PointXYZ> &sphere_cloud, double radius, int number_of_step_alpha=20, int number_of_step_beta=20);

        //void subtractingPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene_ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr  scene_ptr ,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr );

        void addGraspObject(urdf::CollisionSharedPtr grasp_object, std::string parent_link_name);

        void removeGraspObject(const std::string grasp_object_name);

        void removeWorldObject(const std::string world_object_name);

        void pclStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

        void printWorldCollisionObject();

        std::vector< std::pair<std::string, std::string> > getCollisionObjectNames()
        {
            return robot_collision_detector_->getCollisionObjectNames();
        }

/*        bool setJointWeight(const std::vector<manipulator_planner_library::JointWeight> &joints_weight,
                            const std::vector< std::string > &planning_joints_name);
*/
        void setDefaultJointWeight(const std::vector< std::string > &planning_joints_name);
        int& robot_state();
	
	inline void setPlanningGroupName(std::string planning_group_name){planning_group_name_ = planning_group_name;}
	
	std::string getPlanningGroupName(){return planning_group_name_;}
	
	bool getJointLimits(std::vector< double > &lower_limits, std::vector< double > &upper_limits);

	kinematics_library::RobotKinematicsPtr robot_kinematics_;
    private :

        RobotState  robot_state_;
	std::string planning_group_name_;
	    
	
        KDL::Tree kdl_tree_;
        double link_padding_;
        KDL::Chain kdl_chain_;
        boost::shared_ptr<srdf::Model> srdf_model_;
        urdf::ModelInterfaceSharedPtr urdf_model_;
        std::string urdf_file_abs_path_;
        std::string srdf_file_abs_path_;
        std::string ik_fast_shared_object_abs_path_;
		
	collision_detection::AbstractCollisionPtr robot_collision_detector_, world_collision_detector_;
	
	


  
        bool initialiseURDFandSRDF();

        void kdlFrameToEigenMatrix(KDL::Frame &frame,Eigen::Affine3f &transform);

        void scalePointCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                                double scale_x, double scale_y, double scale_z);

        void createPtCloudFromBox(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_box_cloud_ptr,double x, double y, double z,
                                        Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix);

        void createPtCloudFromCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cylinder_cloud_ptr, double radius, double height,
                                                Eigen::Affine3f link_visual_pose_in_sensor_frame_eigen_matrix, int number_of_step_alpha=10 );

        void subtractingPtClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene_ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr  scene_ptr ,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr );
	
	
	
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*these guys are used only in recursive ik solver*/
        /*ikfast::IkSolutionList<IkReal> solutions;
        IkReal eetrans[3];
        IkReal eerot[9];
        bool (*ComputeIk_function_ptr)(const IkReal* , const IkReal* , const IkReal* , ikfast::IkSolutionListBase<IkReal>& );

        std::vector< IkReal>   IKFastSolution;
        std::vector<std::string > mPlanningGroupJointsName;
        bool previous_ik_exist;
        std::vector<IkReal> ikfast_joint_values_previous_result;
        double max_distance_alowed_between_joints_value;
        std::vector<manipulator_planner_library::JointWeight> mJointsWeight;

        bool checkRangeBoundryForJointsFromIKFAST(std::vector<IkReal> joints_values_from_ik_fast);
        bool FindIKSolution(std::vector<RobotFreeJointParameter> &robot_free_joint_parameters,
                                                    std::vector<IkReal> &values_for_free_joints,IkReal free_joint_parameter_values,
                                                    int index);
        bool recursive_nested_loop_with_pivot(std::vector<RobotFreeJointParameter> &robot_free_joint_parameters,
                                                                                    std::vector<IkReal> &values_for_free_joints,
                                                                                    int index);
        bool distance_between_joints_values(std::vector<IkReal> ikfast_joint_values_previous_result,std::vector<IkReal> ikfast_joint_values_from_ik_fast_for_current_pose,
                                            double max_distance_alowed_between_joints_value);

        double calcDistanceBetweenJointsValues( std::vector<IkReal> current_joint_angles, std::vector<IkReal> ik_solution,
                                                const std::vector<manipulator_planner_library::JointWeight>& jt_weight);*/

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        protected:
        void kdl_chain();
};

/*
struct sol_container
{
  std::vector<IkReal> ikfast_sol;
  double distance;
  
};

struct compare_result
{
    bool operator()( sol_container const &a, sol_container const &b) 
    {
      return a.distance < b.distance;
    }

};
*/

}// end namespace motion_planners
#endif // ROBOTMODEL_HPP
