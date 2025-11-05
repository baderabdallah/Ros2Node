#ifndef ROS_NODE_BLUEPRINT_TEST_FIXTURE_HPP
#define ROS_NODE_BLUEPRINT_TEST_FIXTURE_HPP

#include <atomic>
#include <filesystem>
#include <memory>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include "BluePrintNode.hpp"
#include "NodeConstants.h"

namespace blueprint::tests
{

class BluePrintNodeTestFixture : public ::testing::Test
{
protected:
	static void SetUpTestSuite();
	static void TearDownTestSuite();

	void SetUp() override;
	void TearDown() override;

	// Create a helper IO node with pubs/subs and add it to the executor
	void createIoNode(const std::string& node_name);
	// Individual publishers for test inputs
	void publishValueOne(int v);
	void publishValueTwo(float v);
	void publishValueThree(float v);
	// Publish all inputs in one call
	void publishInputs(int one, float two, float three);
	// Spin for one decision cycle: deliver inputs, invoke step(), wait for debug
	void spinForOneDecisionCycle();

	// Simple expectations for blueprint debug message
	void expectFlags(bool inputsFresh, bool valueStable);
	void expectActionPlan(const std::string& expectedCmd, const std::string& expectedReq);

	std::filesystem::path mRunDir;
	rclcpp::NodeOptions mNodeOptions;
	std::shared_ptr<blueprint::BluePrintNode> mNode;
	rclcpp::executors::SingleThreadedExecutor mExecutor;
	// IO helper internals
	rclcpp::Node::SharedPtr mIoNode;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mPubOne;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mPubTwo;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mPubThree;
	rclcpp::Subscription<ros_node_blueprint::msg::MsgOne>::SharedPtr mDebugSub;
	std::shared_ptr<ros_node_blueprint::msg::MsgOne> mLastDebug{};
	std::atomic<uint64_t> mDebugCount{0};
};

} // namespace blueprint::tests

#endif // ROS_NODE_BLUEPRINT_TEST_FIXTURE_HPP
