/** @file CornerHandler.cpp
  * @author Felix Schmoll (LiftnLearn)
  */

/** Includes */
#include "../include/CornerHandler.h"

#include <algorithm>

void CornerHandler::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {

  geometry_msgs::Twist twist = handleCorner(laserScan);
  setTwist(twist);
}

bool CornerHandler::detectCorner(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {

  int numOfValues = (laserScan->angle_min + laserScan->angle_max) /
                    laserScan->angle_increment;

  // what is range_min/range_max?
  float min_distance =
      *std::min_element(laserScan->ranges.begin(), laserScan->ranges.end());

  return (min_distance < CRITICAL_DISTANCE);
}

/** @detail This module is basically just supposed to handle situations when the
 * robot is put into a corner as a starting position. The most basic case here
 * implemented would then be not to bump into the sidewall when turning  away
 * from the same wall.
  */
geometry_msgs::Twist
CornerHandler::handleCorner(const sensor_msgs::LaserScan::ConstPtr &laserScan) {

  geometry_msgs::Twist twist;

  twist.linear.x = twist.linear.y = twist.linear.z = 0;
  twist.angular.x = twist.angular.y = twist.angular.z = 0;

  if (detectCorner(laserScan)) {

    if (laserScan->ranges[0] < CRITICAL_DISTANCE) {
      // angle is such that front is driving away from wall
      if (laserScan->ranges[10] > laserScan->ranges[0]) {

        twist.linear.x = 1;

      } else {

        twist.linear.x = -1;
      }
    } else {

      // angle is such that front is driving away from wall
      if (laserScan->ranges[laserScan->ranges.size() - 11] >
          laserScan->ranges[laserScan->ranges.size() - 1]) {

        twist.linear.x = 1;

      } else {

        twist.linear.x = -1;
      }
    }
  }

  return twist;
}

void CornerHandler::setTwist(geometry_msgs::Twist &twist) {
  this->twist = twist;
}

geometry_msgs::Twist CornerHandler::getTwist() { return twist; }
