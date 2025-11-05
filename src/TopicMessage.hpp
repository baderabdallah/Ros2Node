#ifndef ROS_NODE_BLUEPRINT_TOPIC_MESSAGE_HPP
#define ROS_NODE_BLUEPRINT_TOPIC_MESSAGE_HPP

#include <rclcpp/rclcpp.hpp>

namespace blueprint {

template <typename T> struct TopicMessage {
  T value{};
  bool has{false};
  rclcpp::Time ts{0, 0, RCL_ROS_TIME};
  T last_published{};
  int max_age_ms{0};

  void update(const T &v, const rclcpp::Time &now) {
    value = v;
    has = true;
    ts = now;
  }

  void setMaxAgeMs(int ms) { max_age_ms = ms; }

  bool isFresh(const rclcpp::Time &now) const {
    if (!has)
      return false;
    if (max_age_ms <= 0)
      return true;
    double ms = (now - ts).nanoseconds() / 1e6;
    if (ms < 0.0)
      ms = 0.0;
    return ms <= static_cast<double>(max_age_ms);
  }
};

template <typename... Msgs>
bool allFreshMessages(const rclcpp::Time &now, const Msgs &...msgs) {
  return (... && msgs.isFresh(now));
}

} // namespace blueprint

#endif // ROS_NODE_BLUEPRINT_TOPIC_MESSAGE_HPP
