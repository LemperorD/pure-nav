#pragma once
#include "pure_nav/types.hpp"
#include <string>
namespace pure_nav {
// ROS map_server trinary YAML + PGM, without a ROS runtime. Unknown is blocked.
Grid loadMap(const std::string &yaml_path);
} // namespace pure_nav
