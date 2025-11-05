#include "BluePrintNode.hpp"

#include <cmath>
#include <sstream>

#include <std_msgs/msg/string.hpp>

#include "NodeDebugConversion.hpp"
#include "NodeHelperFunctions.hpp"

using std::placeholders::_1;

namespace blueprint {

BluePrintNode::BluePrintNode(const rclcpp::NodeOptions &options)
    : Node("blueprint_node", options) {
  declareNodeParameters();
  readNodeParameters();

  // Initialize caches max ages
  mValueTwo.setMaxAgeMs(kValueTwoMaxAgeMs);
  mValueOne.setMaxAgeMs(kValueOneMaxAgeMs);
  mValueThree.setMaxAgeMs(kValueThreeMaxAgeMs);

  setupSubscriptions();
  setupPublishers();
  setupTimers();
  setupClients();
  RCLCPP_INFO(this->get_logger(), "blueprint_node initialized");
}

void BluePrintNode::declareNodeParameters() {
  this->declare_parameter(kParamRunDir, std::string{"/tmp/blueprint"});
  this->declare_parameter("enable_decision_timer", true);
}

void BluePrintNode::readNodeParameters() {
  mRunDir = this->get_parameter(kParamRunDir).as_string();
  mEnableDecisionTimer = this->get_parameter("enable_decision_timer").as_bool();
}

void BluePrintNode::setupSubscriptions() {
  // Input One (primary input channel)
  mInputOneSub = createSubscriptionWithLog<std_msgs::msg::Int32>(
      kDefaultInputOneTopic, kDefaultQueueSize,
      std::bind(&BluePrintNode::inputOneCbk, this, _1));

  // Input Two (secondary input channel)
  mInputTwoSub = createSubscriptionWithLog<std_msgs::msg::Float32>(
      kDefaultInputTwoTopic, kDefaultQueueSize,
      std::bind(&BluePrintNode::inputTwoCbk, this, _1));

  // Input Three (tertiary input channel)
  mInputThreeSub = createSubscriptionWithLog<std_msgs::msg::Float32>(
      kDefaultInputThreeTopic, kDefaultQueueSize,
      std::bind(&BluePrintNode::inputThreeCbk, this, _1));
}

void BluePrintNode::setupPublishers() {
  mDebugPublisher = this->create_publisher<ros_node_blueprint::msg::MsgOne>(
      "blueprint/debug", 10);
  RCLCPP_INFO(this->get_logger(), "Set up publisher on topic: blueprint/debug");
}

void BluePrintNode::setupTimers() {
  if (!mEnableDecisionTimer) {
    RCLCPP_INFO(this->get_logger(), "Decision timer disabled by parameter");
    return;
  }
  auto period = std::chrono::duration<double>(1.0 / mDecisionRateHz);
  auto period_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(period);
  mDecisionTimer =
      this->create_wall_timer(period_ms, std::bind(&BluePrintNode::step, this));
}

void BluePrintNode::setupClients() {
  // Pseudocode: set up action/service clients analogous to original code
  // mActionClient = rclcpp_action::create_client<ActionType>(this,
  // kDefaultActionClientName); mServiceClient =
  // this->create_client<ServiceType>(kDefaultServiceClientName);
}

void BluePrintNode::inputOneCbk(const std_msgs::msg::Int32::SharedPtr msg) {
  const auto now = this->now();
  mValueOne.update(msg->data, now);
  mIsStable = isValueStable(mValueOne.value);
  RCLCPP_DEBUG(this->get_logger(), "Cached value_one: %d (stable=%s)",
               mValueOne.value, mIsStable ? "true" : "false");
}

bool BluePrintNode::isValueStable(std::int32_t rawValue) {
  // Pseudocode: insert filter logic here or reuse ModuleOne filter
  (void)rawValue;
  return true; // treat as stable in skeleton
}

void BluePrintNode::inputTwoCbk(const std_msgs::msg::Float32::SharedPtr msg) {
  mValueTwo.update(msg->data, this->now());
  RCLCPP_DEBUG(this->get_logger(), "Cached value_two: %f", msg->data);
}

void BluePrintNode::inputThreeCbk(const std_msgs::msg::Float32::SharedPtr msg) {
  mValueThree.update(msg->data, this->now());
  RCLCPP_DEBUG(this->get_logger(), "Cached value_three: %f", msg->data);
}

void BluePrintNode::step() {
  auto now = this->now();
  ros_node_blueprint::msg::MsgOne msg{};

  // Combined readiness check: require fresh inputs and stable value
  const bool inputsFresh =
      allFreshMessages(now, mValueTwo, mValueOne, mValueThree);
  msg.inputs_fresh = inputsFresh;
  msg.value_stable = mIsStable;

  // Defaults
  msg.command = "NoAction";
  msg.request = "None";

  if (inputsFresh && mIsStable) {
    // Pseudocode: call core logic (ModuleThree Manager) to compute action and
    // request Manager m; m.setX(mValueTwo.value); ... m.execute(); auto plan =
    // m.getPlan(); msg.command = toString(plan.command); msg.request =
    // toString(plan.request);
  }

  mDebugPublisher->publish(msg);

  // Pseudocode: execute plan via action/service clients
  // if (plan.command == Pause) sendPause(); ...
}

} // namespace blueprint
