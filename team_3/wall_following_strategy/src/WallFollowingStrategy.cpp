#include "WallFollowingStrategy.h"

bool WallFollowingStrategy::getCircleVisible() { return circleVisible; }

bool WallFollowingStrategy::getCrashMode() { return crashMode; }

bool WallFollowingStrategy::getCornerHandle() { return cornerStuck; }

void WallFollowingStrategy::getLaserScan(
    const sensor_msgs::LaserScan::ConstPtr &laserScan) {
  lastScan = *laserScan;
}

void WallFollowingStrategy::getCirclePosition(
    const geometry_msgs::Pose2D::ConstPtr &circlePose) {
  if (circlePose->x == -1) {
    circleVisible = false;
    return;
  }

  circleVisible = true;
  circleAngle = circlePose->theta * (180 / M_PI);
  circleDistance = sqrt(pow(circlePose->x, 2) + pow(circlePose->y, 2));
}

float WallFollowingStrategy::findMinimDistance() {
  lastScan.ranges.resize(RANGES);
  float minim = lastScan.ranges[0];

  for (int i = 0; i < RANGES; i++) {
    minim = std::min(minim, lastScan.ranges[i]);
  }

  return minim;
}

void WallFollowingStrategy::getLines(
    const geometry_msgs::Polygon::ConstPtr &lines) {
  lastLineArray = *lines;
}

void WallFollowingStrategy::getCornerRecovery(
    const geometry_msgs::Twist::ConstPtr &cornerOut) {
  if (cornerOut->linear.x == -1) {
    cornerStuck = false;
    return;
  }

  cornerStuck = true;
}

void WallFollowingStrategy::getNearestLineBruteForce() {
  lastScan.ranges.resize(RANGES);
  float error = 0.05;
  float minim = INT_MAX;
  float theta = 90;

  for (int i = 0; i < RANGES - 1; i++) {
    if (abs(lastScan.ranges[i] - lastScan.ranges[i + 1]) < error) {
      minim = std::min(minim, lastScan.ranges[i]);
      theta = i;
    }
  }

  nearestLine.first = minim;

  if (theta >= RANGES / 2) {
    nearestLine.second = theta - RANGES / 2;
  } else {
    nearestLine.second = theta;
  }
}

void correctLeftAlignment() {}

const geometry_msgs::Twist WallFollowingStrategy::controlMovement() {
  geometry_msgs::Twist msg;
  float minim = findMinimDistance();

  if (minim > 0.5) {
    getNearestLineBruteForce();
    msg.linear.x = nearestLine.first;
    msg.angular.z = nearestLine.second;
  }

  if (minim == 0.5) {
    msg.angular.z = 0.5;
    correctLeftAlignment();
  }
}