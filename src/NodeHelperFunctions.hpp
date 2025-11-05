#ifndef ROS_NODE_BLUEPRINT_NODE_HELPER_FUNCTIONS_HPP
#define ROS_NODE_BLUEPRINT_NODE_HELPER_FUNCTIONS_HPP

#include <string>

namespace blueprint::NodeHelperFunctions {

// Pseudocode helpers for building action/service requests
inline std::string createParamArray() { return std::string{"[]"}; }
inline void addModeRequest(std::string &params, int /*mode*/) { (void)params; }
inline void addTargetRequest(std::string &params, float /*x*/, float /*y*/) {
  (void)params;
}
inline void addValueRequest(std::string &params, int /*value*/) {
  (void)params;
}

} // namespace blueprint::NodeHelperFunctions

#endif // ROS_NODE_BLUEPRINT_NODE_HELPER_FUNCTIONS_HPP
