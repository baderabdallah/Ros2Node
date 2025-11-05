#ifndef ROS_NODE_BLUEPRINT_ACTION_PLAN_HPP
+ #define ROS_NODE_BLUEPRINT_ACTION_PLAN_HPP +
    +namespace blueprint::module_three + {
  + +struct ActionPlan + {
    +enum class Command { Pause, Resume, NoAction };
    +enum class Request { TypeA, TypeB, None };
    + +Command command{Command::NoAction};
    +Request request{Request::None};
    +
  };
  + +
}          // namespace blueprint::module_three
+ + #endif // ROS_NODE_BLUEPRINT_ACTION_PLAN_HPP
