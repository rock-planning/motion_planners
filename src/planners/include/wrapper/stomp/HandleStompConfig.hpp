#ifndef HANDLESTOMPCONFIG_HPP_
#define HANDLESTOMPCONFIG_HPP_

#include <string>
#include <fstream>
#include <iostream>
#include <ostream>
#include <yaml-cpp/yaml.h>
#include <stomp/StompConfig.hpp>
#include <abstract/AbstractPlanner.hpp>



namespace handle_stomp_config
{
// void loadConfigFile(std::string filename, YAML::Node &config);
// template<typename T>
// T getValue (const YAML::Node &yaml_data, std::string name);

stomp::StompConfig getStompConfig(const YAML::Node &yaml_data);

stomp::DebugConfig getDebugConfig(const YAML::Node &yaml_data);

}


#endif
