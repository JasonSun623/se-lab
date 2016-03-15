/** @file test_corner_handling.cpp
  * @brief Should test the functionality of overall corner handling.
  * Currently tests for detection of corners and if the minimum distance to the
  *walls decreases in the next step taken.
  *
  * @author Felix Schmoll (LiftnLearn)
  */

/** Includes */

/** Include header for class to be tested */
#include "../include/corner_handling.h"

/** @brief Tests if the node is successfully detecting corners and not giving
 * false positives.
  */
TEST(CornerHandlingTestSuite, cornerDetectionTestCase) {
  CornerHandler handler;
  sensor_msgs::LaserScan laserScan;

  // TODO: fill in actual values in the laserScan (use multiple versions)

  ASSERT(laserScan.detectCorner(laserScan));
}

/** @brief Should somehow test if the distance to the walls is getting bigger/at
 * least not smaller. */
TEST(CornerHandlingTestSuite, wallDistanceTestCase) {}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
