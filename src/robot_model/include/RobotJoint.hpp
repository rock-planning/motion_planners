#ifndef ROBOTJOINT_HPP
#define ROBOTJOINT_HPP


#include <vector>
#include <string>
#include <urdf_model/model.h>

namespace motion_planners
{
/**
 * Struct to hold mimic joint properties
 */
struct MimicJoint
{
    MimicJoint( std::string joint_to_mimic = "", double multiplier = 1, double offset = 0 )
                : joint_to_mimic( joint_to_mimic ), multiplier( multiplier ), offset( offset ) { }

    std::string joint_to_mimic;
    double multiplier;
    double offset;
};


class RobotJoint
{
    
    public:
        RobotJoint(){is_mimic_joint_ = false;};

        inline double getJointValue() {return joint_value_;}

        inline void setJointValue( double joint_value) {joint_value_ = joint_value;}

        inline void setJointName(std::string joint_name) {joint_name_ = joint_name;}

        inline urdf::Joint getJointInfo() {return joint_info_;}

        inline void setJointInfo(urdf::Joint joint_info ) {joint_info_ = joint_info;}

        inline void setJointAsMimic() {is_mimic_joint_ = true;}

        inline bool isMimicJoint( ) { return this->is_mimic_joint_; }

        MimicJoint mimic_joints_;
	std::map< std::string, MimicJoint > mimic_joints_map_; // these joints are mimicking this joint.

	bool is_mimic_joint_;
    private:
        std::string joint_name_;
        double joint_value_;
        urdf::Joint joint_info_;
        

        

};
}// end namespace motion_planners
#endif // ROBOTJOINT_HPP

