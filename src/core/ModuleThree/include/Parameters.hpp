#ifndef ROS_NODE_BLUEPRINT_PARAMETERS_HPP
+ #define ROS_NODE_BLUEPRINT_PARAMETERS_HPP +
    +namespace blueprint::module_three + {
  + +struct Parameters + {
    +bool enableFeature{true};
    +int retryLimit{3};
    +int coolDownSeconds{2};
    +int windowSize{3};
    +int upperTolerance{10};
    +int lowerTolerance{30};
    +
  };
  + +
}          // namespace blueprint::module_three
+ + #endif // ROS_NODE_BLUEPRINT_PARAMETERS_HPP
