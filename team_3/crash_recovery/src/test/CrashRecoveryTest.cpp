#include "../../include/CrashRecoverer.h"

/* Includes */

/* gtest include */
#include <gtest/gtest.h>

/* ROS include */
#include <ros/package.h>

TEST(CrashRecovererTestSuite, crashRecoveryTestCase) {}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
