#include "GenericFilter.hpp"
#include <gtest/gtest.h>

using blueprint::module_one::GenericFilter;

TEST(ModuleOne, Basic) {
  GenericFilter f;
  f.addSample(1.0);
  EXPECT_DOUBLE_EQ(f.getFilteredValue(), 0.0);
}
