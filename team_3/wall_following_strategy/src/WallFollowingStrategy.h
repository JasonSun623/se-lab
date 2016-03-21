#ifndef WALLFOLLOWING_H
#define WALLFOLLOWING_H

#include <ros/ros.h>

/* ROS message type includes */
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>

#include <cmath>
#define RANGES 240

class WallFollowingStrategy {
private:
  bool circleVisible;
  float circleAngle;
  float circleDistance;

  sensor_msgs::LaserScan lastScan;
  geometry_msgs::Polygon lastLineArray;
  std::pair<float, float> nearestLine;

  bool crashMode;
  bool cornerStuck;

public:
  void getCirclePosition(const geometry_msgs::Pose2D::ConstPtr &circlePos);
  void getLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);
  void getCornerRecovery(const geometry_msgs::Twist::ConstPtr &cornerOut);
  bool getCircleVisible();
  bool getCrashMode();
  bool getCornerHandle();
  void getLines(const geometry_msgs::Polygon::ConstPtr &lines);
  void getNearestLineBruteForce();

  float findMinimDistance();

  const geometry_msgs::Twist controlMovement();
};
#endif