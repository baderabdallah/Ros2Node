#ifndef ROS_NODE_BLUEPRINT_BLUEPRINT_NODE_HPP
#define ROS_NODE_BLUEPRINT_BLUEPRINT_NODE_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include "ros_node_blueprint/msg/msg_one.hpp"

#include "NodeConstants.h"
#include "TopicMessage.hpp"

namespace blueprint {

class BluePrintNode : public rclcpp::Node {
public:
  explicit BluePrintNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void step();

private:
  // subscribers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
      mInputOneSub; // primary input channel
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      mInputTwoSub; // secondary input channel
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      mInputThreeSub; // tertiary input channel

  // clients (placeholders; types will be pseudocode)
  // rclcpp_action::Client<ActionType>::SharedPtr mActionClient{}; // pseudo
  // rclcpp::Client<ServiceType>::SharedPtr mServiceClient{}; // pseudo

  // publishers
  rclcpp::Publisher<ros_node_blueprint::msg::MsgOne>::SharedPtr
      mDebugPublisher{};

  // timers
  rclcpp::TimerBase::SharedPtr mDecisionTimer{};

  // caches
  TopicMessage<std::int32_t> mValueOne{}; // input one
  TopicMessage<float> mValueTwo{};        // input two
  TopicMessage<float> mValueThree{};      // input three
  bool mIsStable{false};

  // parameters
  double mDecisionRateHz{2.0};
  bool mEnableDecisionTimer{true};
  std::string mRunDir{};

private:
  void declareNodeParameters();
  void readNodeParameters();
  void setupSubscriptions();
  void setupPublishers();
  void setupTimers();
  void setupClients();

  void inputOneCbk(const std_msgs::msg::Int32::SharedPtr msg);
  void inputTwoCbk(const std_msgs::msg::Float32::SharedPtr msg);
  void inputThreeCbk(const std_msgs::msg::Float32::SharedPtr msg);

  bool isValueStable(std::int32_t rawValue);

  template <typename MsgT, typename CallbackT>
  typename rclcpp::Subscription<MsgT>::SharedPtr
  createSubscriptionWithLog(const std::string &topic, int queueSize,
                            CallbackT callback) {
    auto sub = this->create_subscription<MsgT>(topic, queueSize, callback);
    RCLCPP_INFO(this->get_logger(), "Set up subscription on topic: %s",
                topic.c_str());
    return sub;
  }
};

} // namespace blueprint

#endif // ROS_NODE_BLUEPRINT_BLUEPRINT_NODE_HPP
