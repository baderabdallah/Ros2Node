#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "BluePrintNode.hpp"
#include "BluePrintNodeTestFixture.hpp"

using blueprint::tests::BluePrintNodeTestFixture;

TEST_F(BluePrintNodeTestFixture, NodeConstructs)
{
  auto node = std::make_shared<blueprint::BluePrintNode>();
  ASSERT_NE(node, nullptr);
}

TEST_F(BluePrintNodeTestFixture, PublishesDebugOnInputs)
{
  createIoNode("bp_test_io");
  publishInputs(10, 2.0f, 1.0f);
  spinForOneDecisionCycle();
  expectFlags(true, true); // skeleton treats stability as true
}

TEST_F(BluePrintNodeTestFixture, DefaultActionPlanStrings)
{
  createIoNode("bp_test_io2");
  publishInputs(5, 1.0f, 0.5f);
  spinForOneDecisionCycle();
  expectActionPlan("NoAction", "None");
}
