/** @file RandomWalkStrategy.cpp
  * Implementation of the corresponding header.
  *
  * @author Leonhard Kuboschek
  * @bug Sometimes drives into walls if placed closely to multiple walls.
*/

#include "RandomWalkStrategy.h"

/**
  * @brief Returns if circles is in sight.
  * @return Returns bool if circle is visible.
  */
bool RandomWalkStrategy::getCircleVisible() { return circleVisible; }

/**
  * @brief Checks if pose represents actual circle and sets values accordingly.
  */
void RandomWalkStrategy::receiveCirclePosition(
    const geometry_msgs::Pose2D::ConstPtr &circlePose) {
  // compare to -1
  if (abs(circlePose->x + 1) < EPSILON) {
    circleVisible = false;
    return;
  }

  circleVisible = true;
  circleAngle = circlePose->theta * (180 / M_PI);
  circleDistance = sqrt(pow(circlePose->x, 2) + pow(circlePose->y, 2));
}

void RandomWalkStrategy::receiveLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  lastScan = *laserScan;
}

/**
  * @brief Just loops over all distances and finds the minimum.
  */
float RandomWalkStrategy::findMinim(int num_readings) {
  lastScan.ranges.resize(num_readings);
  float minim = lastScan.ranges[0];

  for (int i = 0; i < num_readings; i++) {
    minim = std::min(minim, lastScan.ranges[i]);
  }

  return minim;
}

/**
 * @brief If circle is visible it drives straight towards it. If not it moves
 * either forward or turns if it gets too close to a wall.
 * @return Next move to be done.
 */
const geometry_msgs::Twist RandomWalkStrategy::getControlOutput() {
  geometry_msgs::Twist msg;

  if (circleVisible) {
    float variation = SCAN_CENTER - circleAngle;

    if (abs(variation) > VARIATION_THRESHOLD)
      correcting = true;

    // stop correting if alignment sufficient
    if (abs(variation) < VARIATION_THRESHOLD - HYSTERESIS)
      correcting = false;

    ROS_DEBUG("V: %f\tC: %s", variation, correcting ? "true" : "false");

    if (correcting) {
      // turn
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
