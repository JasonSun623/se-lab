#include "WallFollowingStrategy.h"

bool WallFollowingStrategy::getCircleVisible() { return circleVisible; }

bool WallFollowingStrategy::getCrashMode() { return crashMode; }

bool WallFollowingStrategy::getCornerHandle() { return cornerStuck; }

void WallFollowingStrategy::getLaserScan(
    const geometry_msgs::LaserScan::ConstPtr &laserScan) {
  lastScan = *laserScan;
}

void WallFollowingStrategy::getCirclePosition(
    const geometry_msgs::Pose2D::ConstPtr &circlePos) {
  if (circlePos->x == -1) {
    circleVisible = false;
    return;
  }

  circleVisible = true;
  circleAngle = circlePose->theta * (180 / M_PI);
  circleDistance = sqrt(pow(circlePose->x, 2) + pow(circlePose->y, 2));
}

float WallFollowingStrategy::findMinim(int num_readings) {
  lastScan.ranges.resize(num_readings);
  float minim = lastScan.ranges[0];

  for (int i = 0; i < num_readings; i++) {
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

const geometry_msgs::Twist WallFollowingStrategy::controlMovement() {
  geometry_msgs::Twist msg;
}