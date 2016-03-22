/** @file CornerHandler.cpp
  * @author Felix Schmoll (LiftnLearn)
  */

/** Includes */
#include "../include/CornerHandler.h"

#include <algorithm>

bool CornerHandler::detectCorner(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {

  int numOfValues = (laserScan->angle_min + laserScan->angle_max) /
                    laserScan->angle_increment;

  // what is range_min/range_max?
  float min_distance =
      *std::min_element(laserScan->ranges.begin(), laserScan->ranges.end());

  return (min_distance < CRITICAL_DISTANCE);
}

geometry_msgs::Twist handleCorner(sensor_msgs::LaserScan &laserScan) {

  geometry_msgs::Twist twist;
  return twist;
}
