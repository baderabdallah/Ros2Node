#ifndef ROS_NODE_BLUEPRINT_GENERIC_FILTER_HPP
+ #define ROS_NODE_BLUEPRINT_GENERIC_FILTER_HPP +
    +namespace blueprint::module_one + {
  + +class GenericFilter + {
    +public : +GenericFilter() = default;
    +void addSample(double /*x*/) {
    }
    +double getFilteredValue() const {
      return 0.0;
    }
    +
  };
  + +
}          // namespace blueprint::module_one
+ + #endif // ROS_NODE_BLUEPRINT_GENERIC_FILTER_HPP
