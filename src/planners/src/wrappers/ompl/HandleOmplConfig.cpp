#include <wrapper/ompl/HandleOmplConfig.hpp>
#include <wrapper/ompl/OmplConfig.hpp>

namespace YAML 
{
template<>
struct convert<ompl_config::PlannerType> 
{
    static Node encode(const ompl_config::PlannerType& plannertype) 
    {
        Node node;
        node.push_back(plannertype);   
        return node;
    }

    static bool decode(const Node& node, ompl_config::PlannerType& plannertype) 
    {
        if(!node.IsScalar())
            return false;  

        std::unordered_map<std::string, ompl_config::PlannerType> stringToenum;
        stringToenum.insert({":SBL", ompl_config::SBL});
        stringToenum.insert({":EST", ompl_config::EST});
        stringToenum.insert({":LBKPIECE", ompl_config::LBKPIECE});
        stringToenum.insert({":BKPIECE", ompl_config::BKPIECE});
        stringToenum.insert({":KPIECE", ompl_config::KPIECE});
        stringToenum.insert({":RRT", ompl_config::RRT});
        stringToenum.insert({":RRTConnect", ompl_config::RRTConnect});
        stringToenum.insert({":RRTstar", ompl_config::RRTstar});
        stringToenum.insert({":PRM", ompl_config::PRM});
        stringToenum.insert({":PRMstar", ompl_config::PRMstar});

        LOG_DEBUG_S<<"[getOmplConfig]: Selected planner = "<<stringToenum.at(node.Scalar());
        plannertype  = stringToenum.at(node.Scalar());
        return true;
    }
};
}

namespace handle_ompl_config
{
ompl_config::OmplConfig getOmplConfig(const YAML::Node &yaml_data)
{
    ompl_config::OmplConfig config;    

    try 
    {
        config.type_of_planner = (ompl_config::PlannerType) motion_planners::getValue<int, double>(yaml_data, "type_of_planner");
    } 
    catch (std::exception) 
    {
        config.type_of_planner = motion_planners::getValue<ompl_config::PlannerType>(yaml_data, "type_of_planner");
    }
    config.max_solve_time                   = motion_planners::getValue<double>(yaml_data, "max_solve_time");
    config.max_step_smoothing               = motion_planners::getValue<unsigned int, double>(yaml_data, "max_step_smoothing");    
    try 
    {
        config.planner_specific_parameters_range = motion_planners::getValue<std::string>(yaml_data, "planner_specific_parameters_range");
    }
    catch (std::exception) 
    {
        config.planner_specific_parameters_range = std::to_string(motion_planners::getValue<int, double>(yaml_data, "planner_specific_parameters_range"));
    }
    return config;
}
}
