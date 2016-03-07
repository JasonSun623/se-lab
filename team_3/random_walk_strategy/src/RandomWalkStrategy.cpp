#include "RandomWalkStrategy.h"
#include <geometry_msgs/Pose2D.h>

// Number of laser samples
#define RANGES 250

bool RandomWalkStrategy::getCircleVisible() { return circleVisible; }

void RandomWalkStrategy::receiveCirclePosition(
    const geometry_msgs::Pose2D::ConstPtr &circlePose) {
  if (circlePose->x == -1) {
    circleVisible = false;
    return;
  }

  circleVisible = true;
  circleAngle = circlePose->theta * (180 / M_PI);
  circleDistance = sqrt(pow(circlePose->x, 2) + pow(circlePose->y, 2));

  // ROS_INFO("Circle angle: %f", circleAngle);
  // ROS_INFO("Circle distance: %f", circleDistance);
}

void RandomWalkStrategy::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  lastScan = *laserScan;
}

float RandomWalkStrategy::findMinim(int num_readings) {
  lastScan.ranges.resize(num_readings);
  float minim = lastScan.ranges[0];

  for (int i = 0; i < num_readings; i++) {
    minim = std::min(minim, lastScan.ranges[i]);
  }

  return minim;
}

const geometry_msgs::Twist RandomWalkStrategy::getControlOutput() {
  geometry_msgs::Twist msg;

  if (circleVisible) {
    float variation = SCAN_CENTER - circleAngle;

    if (abs(variation) > VARIATION_THRESHOLD)
      correcting = true;

    if (abs(variation) < VARIATION_THRESHOLD - HYSTERESIS)
      correcting = false;

    // ROS_INFO("V: %f\tC: %s", variation, correcting ? "true" : "false");

    if (correcting) {
      msg.angular.z = std::min(MAX_TURN, TURN_CORRECTION * variation);
      msg.linear.x = LINEAR_VEL / std::min(4.0, abs(variation) / 10.0);
    } else {
      msg.linear.x = LINEAR_VEL;
    }
  } else {
    float min = findMinim(RANGES);

    if (min > MIN_DISTANCE) {
      msg.linear.x = LINEAR_VEL / 2;
    } else {
      msg.linear.x = 0;
      msg.angular.z = SCAN_VELOCITY;
    }
  }

  return msg;
}
