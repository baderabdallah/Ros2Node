#include "Manager.hpp"
#include <gtest/gtest.h>

using blueprint::module_three::Manager;

TEST(ModuleThree, Basic) {
  Manager m;
  m.execute();
  auto plan = m.getPlan();
  (void)plan;
  SUCCEED();
}
