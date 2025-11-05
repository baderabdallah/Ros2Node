#ifndef ROS_NODE_BLUEPRINT_MANAGER_HPP
+ #define ROS_NODE_BLUEPRINT_MANAGER_HPP +
    +#include<cstdint> + +#include "ActionPlan.hpp" +
    #include "Parameters.hpp" + +namespace blueprint::module_three + {
  + +class Manager +
      {+public : +void setConfig(const Parameters &){} +
       void setValueOne(std::int32_t){} + void setValueTwo(float){} +
       void setValueThree(float){} + void execute(){} +
       ActionPlan getPlan() const {return ActionPlan{};
}
+ std::int32_t getLastAcceptedValue()const {
  return 0;
}
+
}
;
+ +
}          // namespace blueprint::module_three
+ + #endif // ROS_NODE_BLUEPRINT_MANAGER_HPP
