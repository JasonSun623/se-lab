#ifndef WALLFOLLOWING_H
#define WALLFOLLOWING_H

#include <ros/ros.h>

/* ROS message type includes */
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <assert.h>
#include <vector>
#include <algorithm>
#include <cmath>

#include <cmath>

#define RANGES 240

#define LASER_RANGE 3.9

#define LINEAR_VEL 1.0

/**
 * @brief The amount of variation that is allowed before direction is corrected
 * [deg]
 */
#define VARIATION_THRESHOLD 8

/**
  * @brief The value for which realignment is being done. If the value is
 * smaller we don't correct anymore. [deg]
  */
#define HYSTERESIS 2

/**
 * @brief Angular velocity value that is used to turn
 */
#define TURN_CORRECTION 0.05

/**
  * @brief Upper speed for turning rate
  */
#define MAX_TURN 0.8

/**
 * @brief Angular velocity for scanning for circle
 */
#define SCAN_VELOCITY 0.5

/**
 * @brief Minimum allowed distance to obstacles
 */
#define MIN_DISTANCE 0.5

class WallFollowingStrategy {
private:
  bool circleVisible;
  float circleAngle;
  float circleDistance;
  float robotAngle;

  geometry_msgs::Polygon lastLineArray;
  std::pair<float, float> nearestLine;

  bool crashMode;
  bool cornerStuck;
  bool followWall;

  std::vector<cv::Vec4i> res;
  std::vector<std::pair<float, float>> initialLineChoice;
  std::vector<std::pair<int, int>> points;
  sensor_msgs::LaserScan lastScan;
  cv::Mat src;

public:
  void getCirclePosition(const geometry_msgs::Pose2D::ConstPtr &circlePos);
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);
  void getCornerRecovery(const geometry_msgs::Twist::ConstPtr &cornerOut);
  bool getCircleVisible();
  bool getCrashMode();
  bool getCornerHandle();
  void getLines(const geometry_msgs::Polygon::ConstPtr &lines);
  void getNearestLineBruteForce();
  static int compareStart(cv::Vec4i one, cv::Vec4i two) {
    return (one[0] < two[0]);
  }

  static int compareEnd(std::pair<cv::Vec4i, float> one,
                        std::pair<cv::Vec4i, float> two) {
    return (one.first[2] > two.first[2]);
  }

  static int compareSlope(std::pair<cv::Vec4i, float> one,
                          std::pair<cv::Vec4i, float> two) {
    return (one.second < two.second);
  }

  static int compareDist(std::pair<float, float> one,
                         std::pair<float, float> two) {
    return (one.first < two.first);
  }

  int compareY(int y1, int y2) {
    if (y1 > y2) {
      return (y1 - y2);
    } else {
      return (y2 - y1);
    }
  }

  float calcSlope(cv::Vec4i);
  cv::Vec4i getAverSlope(std::vector<std::pair<cv::Vec4i, float>>);
  int getDifference(int, int);
  void printLinesImage(cv::Mat, std::vector<cv::Vec4i>);
  void removeLines(std::vector<cv::Vec4i>);
  cv::Mat
  createOpenCVImageFromLaserScan(const sensor_msgs::LaserScan::ConstPtr &);
  float interpolate(int, int, std::vector<float>);
  int getNumLines() { return res.size(); }
  sensor_msgs::LaserScan getLaserScan() const { return lastScan; }
  cv::Mat getImage() { return src; }

  void clearData() { res.clear(); }

  void setImage(cv::Mat src) { this->src = src; }

  std::vector<cv::Vec4i> getLines() { return res; }

  std::pair<float, float> findMinimDistance(int, int);

  const geometry_msgs::Twist controlMovement();
  float getCurrentAngle() { return robotAngle; }
  void setCurrentAngle(float angle) { robotAngle = angle; }
  geometry_msgs::Twist turnLeft();
};
#endif