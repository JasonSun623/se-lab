/** @file CrashRecoverer.cpp
  * @brief Standard implementation of the header.
  *
  * @author Leonhard Kuboschek
  */
#include "CrashRecoverer.h"

#define CRASH_DISTANCE 20 // In mm

#define RECOVERY_STEPS 20 // Number of steps to the recovery

// Fixed sequence of linear velocities
float linVelocities [RECOVERY_STEPS] = {
  -0.2, -0.3, -0.5, -0.7, -0.7,
  -0.7, -0.7, -0.5, -0.3, -0.2
};

// Fixed sequence of angular velocities
float angVelocities [RECOVERY_STEPS] = {
  0.0, 0.0, 0.0, 0.2, 0.4,
  0.4, 0.4, 0.4, 0.2, 0.0
};

void CrashRecoverer::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  lastScan = *laserScan;
}

float CrashRecoverer::findMinim(int num_readings) {
  lastScan.ranges.resize(num_readings);
  float minim = lastScan.ranges[0];

  for (int i = 0; i < num_readings; i++) {
    minim = std::min(minim, lastScan.ranges[i]);
  }

  return minim;
}

const geometry_msgs::Twist CrashRecoverer::getControlOutput() {
  geometry_msgs::Twist msg;

  float min = findMinim(RANGES);

  if(min < CRASH_DISTANCE) {
    // Crash condition, change state
    currentState = CRASH;
    recoveryTimer = 0;
  }

  if(currentState == CRASH) {
    if(recoveryTimer == RECOVERY_STEPS - 1) {
      currentState = OK;
    } else {
      msg.linear.x = linVelocities[recoveryTimer];
      msg.angular.z = angVelocities[recoveryTimer];
      recoveryTimer++;
    }
  }

  return msg;
}
