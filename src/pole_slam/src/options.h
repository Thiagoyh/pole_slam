#pragma once


#include <string>
#include "common/lua_parameter_dictionary.h"
#include "common/port.h"

namespace config_lua{

struct NodeOptions{
    // the config of spherical conversion
    double fov_up;
    double fov_down;
    int img_height;
    int img_width;

    // the config of cluster
    double min_cluster_dis = 0.08;
    int min_cluster_num = 3;
    int width_thr = 10;
};

NodeOptions CreateNodeOptions(
    common::LuaParameterDictionary* lua_parameter_dictionary);

NodeOptions LoadOptions(const std::string &configuration_directory,
                        const std::string &configuration_basename);
}