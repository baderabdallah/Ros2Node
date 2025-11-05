#ifndef ROS_NODE_BLUEPRINT_NODE_CONSTANTS_H
#define ROS_NODE_BLUEPRINT_NODE_CONSTANTS_H

#include <string>

// defaults
inline constexpr int kDefaultQueueSize{10};

inline const std::string kDefaultInputOneTopic{"/blueprint/input_one"};
inline const std::string kDefaultInputTwoTopic{"/blueprint/input_two"};
inline const std::string kDefaultInputThreeTopic{"/blueprint/input_three"};

inline const std::string kDefaultActionClientName{"/blueprint/handle_tasks"};
inline const std::string kDefaultServiceClientName{"/blueprint/pause_resume"};

// ROS parameters keys
inline const std::string kParamRunDir{"run_dir"};

// Max ages for freshness gating
inline constexpr int kValueTwoMaxAgeMs = 5000;  // secondary input max age
inline constexpr int kValueOneMaxAgeMs = 1000;  // primary input max age
inline constexpr int kValueThreeMaxAgeMs = 500; // tertiary input max age

#endif // ROS_NODE_BLUEPRINT_NODE_CONSTANTS_H
