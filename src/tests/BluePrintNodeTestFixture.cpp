#include "BluePrintNodeTestFixture.hpp"

#include <chrono>
#include <thread>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

namespace
{
constexpr const char kRunDirFolderName[] = "run";
}

namespace blueprint::tests
{

void BluePrintNodeTestFixture::SetUpTestSuite()
{
	int argc = 0; char** argv = nullptr;
	if (!rclcpp::ok()) rclcpp::init(argc, argv);
}

void BluePrintNodeTestFixture::TearDownTestSuite()
{
	if (rclcpp::ok()) rclcpp::shutdown();
}

void BluePrintNodeTestFixture::SetUp()
{
	namespace fs = std::filesystem;
	fs::path this_file = fs::path(__FILE__).lexically_normal();
	mRunDir = this_file.parent_path() / kRunDirFolderName;
	mNodeOptions = rclcpp::NodeOptions{};
	mNodeOptions.append_parameter_override("run_dir", mRunDir.string());
	// Disable timer for deterministic tests
	mNodeOptions.append_parameter_override("enable_decision_timer", false);

	mNode = std::make_shared<blueprint::BluePrintNode>(mNodeOptions);
	mExecutor.add_node(mNode);
}

void BluePrintNodeTestFixture::TearDown()
{
	if (mIoNode)
	{
		mExecutor.remove_node(mIoNode);
		mDebugSub.reset();
		mPubOne.reset();
		mPubTwo.reset();
		mPubThree.reset();
		mIoNode.reset();
		mLastDebug.reset();
	}
	mExecutor.remove_node(mNode);
	mNode.reset();
}

void BluePrintNodeTestFixture::createIoNode(const std::string& node_name)
{
	mIoNode = std::make_shared<rclcpp::Node>(node_name);
	mExecutor.add_node(mIoNode);

	constexpr int qos_depth = 10;
	mPubOne = mIoNode->create_publisher<std_msgs::msg::Int32>(kDefaultInputOneTopic, qos_depth);
	mPubTwo = mIoNode->create_publisher<std_msgs::msg::Float32>(kDefaultInputTwoTopic, qos_depth);
	mPubThree = mIoNode->create_publisher<std_msgs::msg::Float32>(kDefaultInputThreeTopic, qos_depth);

	mDebugSub = mIoNode->create_subscription<ros_node_blueprint::msg::MsgOne>(
		"blueprint/debug", qos_depth,
		[&](ros_node_blueprint::msg::MsgOne::SharedPtr msg)
		{
			mLastDebug = std::make_shared<ros_node_blueprint::msg::MsgOne>(*msg);
			++mDebugCount;
		});

	for (int i = 0; i < 3; ++i)
	{
		mExecutor.spin_some();
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void BluePrintNodeTestFixture::publishValueOne(int v)
{
	std_msgs::msg::Int32 m; m.data = v; mPubOne->publish(m);
}

void BluePrintNodeTestFixture::publishValueTwo(float v)
{
	std_msgs::msg::Float32 m; m.data = v; mPubTwo->publish(m);
}

void BluePrintNodeTestFixture::publishValueThree(float v)
{
	std_msgs::msg::Float32 m; m.data = v; mPubThree->publish(m);
}

void BluePrintNodeTestFixture::publishInputs(int one, float two, float three)
{
	publishValueOne(one);
	publishValueTwo(two);
	publishValueThree(three);
}

void BluePrintNodeTestFixture::spinForOneDecisionCycle()
{
	for (int i = 0; i < 2; ++i)
	{
		mExecutor.spin_some();
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}

	mNode->step();

	const auto startCount = mDebugCount.load();
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
	while (mDebugCount.load() == startCount && std::chrono::steady_clock::now() < deadline)
	{
		mExecutor.spin_some();
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
}

void BluePrintNodeTestFixture::expectFlags(bool inputsFresh, bool valueStable)
{
	ASSERT_NE(mLastDebug, nullptr);
	EXPECT_EQ(mLastDebug->inputs_fresh, inputsFresh);
	EXPECT_EQ(mLastDebug->value_stable, valueStable);
}

void BluePrintNodeTestFixture::expectActionPlan(const std::string& expectedCmd, const std::string& expectedReq)
{
	ASSERT_NE(mLastDebug, nullptr);
	EXPECT_EQ(mLastDebug->command, expectedCmd);
	EXPECT_EQ(mLastDebug->request, expectedReq);
}

} // namespace blueprint::tests

