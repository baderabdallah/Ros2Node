#include "ValueHandlerImpl.hpp"
#include <gtest/gtest.h>

using blueprint::module_two::ValueHandlerImpl;

TEST(ModuleTwo, Basic) {
  ValueHandlerImpl h;
  h.setTolerance(10, 20);
  EXPECT_FALSE(h.isOutOfRange(15));
}
