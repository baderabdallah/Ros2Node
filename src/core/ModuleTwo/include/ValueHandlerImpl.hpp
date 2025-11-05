#ifndef ROS_NODE_BLUEPRINT_VALUE_HANDLER_IMPL_HPP
+ #define ROS_NODE_BLUEPRINT_VALUE_HANDLER_IMPL_HPP +
    +namespace blueprint::module_two + {
  + +class ValueHandlerImpl +
      {+public : +void setTolerance(int /*lower*/, int /*upper*/){} +
       bool isOutOfRange(int /*value*/) const {return false;
}
+
}
;
+ +
}          // namespace blueprint::module_two
+ + #endif // ROS_NODE_BLUEPRINT_VALUE_HANDLER_IMPL_HPP
