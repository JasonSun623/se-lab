/** @file CrashRecoverer.cpp
  * @brief Standard implementation of the header.
  *
  * @author Leonhard Kuboschek
  */

#include <algorithm>

#include "../include/CrashRecoverer.h"

/**
  * @brief If any point in the laser scan is below this distance, we've crashed
  */
#define CRASH_DISTANCE 0.05

/**
  * @brief The length of the recovery maneuver in 'steps'. One step is equal
  * to one iteration of this package's node's loop.
  */
#define RECOVERY_STEPS 20

/**
  * Fixed sequence of linear velocities
  */
float linVelocities [RECOVERY_STEPS] = {
  -0.2, -0.3, -0.5, -0.7, -0.7,
  -0.7, -0.7, -0.5, -0.3, -0.2
};

/**
  * Fixed sequence of angular velocities
  */
float angVelocities [RECOVERY_STEPS] = {
  0.0, 0.0, 0.0, 0.2, 0.4,
  0.4, 0.4, 0.4, 0.2, 0.0
};

void CrashRecoverer::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  geometry_msgs::Twist msg;

  float min = *std::min_element(laserScan->ranges.begin(), laserScan->ranges.end());

  if (min < CRASH_DISTANCE) {
    // Crash condition, change state
    currentState = CRASH;
    recoveryTimer = 0;
  }

  if (currentState == CRASH) {
    if (recoveryTimer == RECOVERY_STEPS - 1) {
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
