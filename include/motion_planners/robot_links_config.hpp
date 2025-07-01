#ifndef ROBOT_LINKS_CONFIG_HPP
#define ROBOT_LINKS_CONFIG_HPP

#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>

namespace motion_planners {

class RobotConfig {
public:
    struct RobotLinks {
        std::string base;
        std::string si;
        std::string j0;
        std::string ee;
        std::string j1, j2, j3, j4, j5, j6, j7;
    };

    struct WorldLinks {
        std::string name;
    };

    RobotLinks kuka;
    RobotLinks vispa;
    WorldLinks world;

    void load_from_yaml(const std::string& path) {
        std::cout << "Loading robot links from: " << path << std::endl;
        YAML::Node config = YAML::LoadFile(path);

        auto k = config["kuka"];
        if (k) {
            kuka.base = k["base"].as<std::string>();
            kuka.si   = k["si"].as<std::string>();
            kuka.j0   = k["j0"].as<std::string>();
            kuka.ee   = k["ee"].as<std::string>();
            kuka.j1   = k["j1"].as<std::string>();
            kuka.j2   = k["j2"].as<std::string>();
            kuka.j3   = k["j3"].as<std::string>();
            kuka.j4   = k["j4"].as<std::string>();
            kuka.j5   = k["j5"].as<std::string>();
            kuka.j6   = k["j6"].as<std::string>();
            kuka.j7   = k["j7"].as<std::string>();
        }

        auto v = config["vispa"];
        if (v) {
            vispa.base = v["base"].as<std::string>();
            vispa.si   = v["si"].as<std::string>();
            vispa.j0   = v["j0"].as<std::string>();
            vispa.ee   = v["ee"].as<std::string>();
            vispa.j1   = v["j1"].as<std::string>();
            vispa.j2   = v["j2"].as<std::string>();
            vispa.j3   = v["j3"].as<std::string>();
            vispa.j4   = v["j4"].as<std::string>();
            vispa.j5   = v["j5"].as<std::string>();
            vispa.j6   = v["j6"].as<std::string>();
        }

        auto w = config["world"];
        if (w) {
            world.name = w["name"].as<std::string>();
        }
    }
};

} // namespace motion_planners

#endif // ROBOT_LINKS_CONFIG_HPP
