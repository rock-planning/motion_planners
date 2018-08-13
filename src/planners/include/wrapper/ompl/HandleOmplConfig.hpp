#ifndef HANDLEOMPLCONFIG_HPP_
#define HANDLEOMPLCONFIG_HPP_

#include <string>
#include <fstream>
#include <iostream>
#include <ostream>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <abstract/AbstractPlanner.hpp>
#include <wrapper/ompl/OmplConfig.hpp>


namespace handle_ompl_config
{
    ompl_config::OmplConfig getOmplConfig(const YAML::Node &yaml_data);
}
#endif
