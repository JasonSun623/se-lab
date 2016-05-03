/** @file CrashRecoverer.cpp
  * @brief Standard implementation of the header.
  *
  * @author Leonhard Kuboschek (kuboschek)
  */

#include <algorithm>

#include "../include/CrashRecoverer.h"


/**
 * Fixed sequence of linear velocities.
 */
float linVelocities[] = {-0.2, -0.3, -0.5, -0.7, -0.7,
                                       -0.7, -0.7, -0.5, -0.3, -0.2};

/**
 * Fixed sequence of angular velocities.
 */
float angVelocities[] = {0.0, 0.0, 0.0, 0.2, 0.4,
                                       0.4, 0.4, 0.4, 0.2, 0.0};

CrashRecoverer::CrashRecoverer(float crash_distance, int recovery_steps) {
  this->crash_distance = crash_distance;
  this->recovery_steps = recovery_steps;
}

void CrashRecoverer::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  geometry_msgs::Twist msg;

  float min =
      *std::min_element(laserScan->ranges.begin(), laserScan->ranges.end());

  if (min < crash_distance) {
    // Crash condition, change state
    currentState = CRASH;
    recoveryTimer = 0;
  }

  if (currentState == CRASH) {
    if (recoveryTimer == recovery_steps - 1) {
      currentState = OK;
    } else {
      // Replay 'stored' procedure to robot
      msg.linear.x = linVelocities[recoveryTimer];
      msg.angular.z = angVelocities[recoveryTimer];

      recoveryTimer++;
    }
  }

  currentOut = msg;
}

const geometry_msgs::Twist CrashRecoverer::getResolution() {
  return currentOut;
}
