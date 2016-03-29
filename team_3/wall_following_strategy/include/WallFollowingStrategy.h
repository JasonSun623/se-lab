/** @file WallFollowingStrategy.h
  * Implementation of wall-following strategy
  * The robot starts to move straight until it finds a wall
  * Using right-hand rule robot follows the wall and try to avoid
  * single walls, corners by not touching them
  * @author Mariia Gladkova
  * @author Felix Schmoll
  */

#ifndef WALLFOLLOWING_H
#define WALLFOLLOWING_H

/** Includes */

/** include ROS */
#include <ros/ros.h>
#include <ros/package.h>

/** include messages */
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>

/** include OpenCV */
#include <opencv2/opencv.hpp>

/** include C library */
#include <assert.h>
#include <vector>
#include <algorithm>
#include <cmath>

#define RANGES 250

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
#define TURN_CORRECTION 0.01

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

/**
  * @brief Limiting integers to be within a certain range.
  */
#define RANGE(l, x, r) (std::max((l), std::min((r), (x))))

/**
  * @brief Factor to define an image dimensions
  */

#define STRETCH_FACTOR 100

/**
  * @brief Compound class for wall-following strategy implementation
  */

class WallFollowingStrategy {
private:
  bool circleVisible;
  float circleAngle;
  float circleDistance;
  float robotAngle;

  bool crashMode;
  bool cornerStuck;
  bool followWall;
  bool circleFoundMode;
  bool correcting;
  bool cornerEdge;

  std::vector< cv::Vec4i > res;
  std::vector< std::pair< float, float > > initialLineChoice;
  std::vector< std::pair< int, int > > points;
  sensor_msgs::LaserScan lastScan;
  cv::Mat src;
  geometry_msgs::Twist cornerHandler;
  geometry_msgs::Twist crashHandler;

public:
  /**
  * @brief Gets position of a circle if detected as a Pose2D message
  * @param circlePos Pose2D message about location of the circle(x,y coordinates
  * and angle)
  */
  void receiveCirclePosition(const geometry_msgs::Pose2D::ConstPtr &circlePos);

  /**
  * @brief Receives a laser scan message and creates an OpenCV image
  * @param laserScan LaserScan Message with information about the distances
  */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /**
  * @brief Receives a message from corner_handling topic of robot behavior
  * in case of detecting a corner or an obstacle
  * @detail Slowly moves the robot out of the corner or in the opposite to
  * the wall direction without touching touching other obstacles/walls.
  */
  void getCornerRecovery(const geometry_msgs::Twist::ConstPtr &cornerOut);

  /**
  * @brief Receives a message from crash_recovery topic of robot behavior
  * in case of crashing into the obstacle
  * @detail Slowly moves the robot opposite to
  * the wall direction without touching other obstacles/walls.
  */
  void getCrashRecovery(const geometry_msgs::Twist::ConstPtr &crashOut);

  /**
  * @brief Returns whether a circle has been detected or not
  */
  bool getCircleVisible();
  /**
  * @brief Returns whether a robot crashed into the obstacle or not
  */
  bool getCrashMode();
  /**
  * @brief Returns whether a robot has stuck in a corner or not
  */
  bool getCornerHandle();

  /**
  * @brief Comparison function for std::sort
  * @detail Compares the x-coordinates of starting points of the line segments
  * @param one, two Starting and ending position coordinates of line segments
  * @return A value that is convertible to bool and shows the order of two
  * coordinates
  */
  static int compareStart(cv::Vec4i one, cv::Vec4i two) {
    return (one[0] < two[0]);
  }

  /**
  * @brief Comparison function for std::sort
  * @detail Compares the x-coordinates of ending points of the line segments
  * @param one, two Vectors with 4 parameters (Starting and ending position
  * coordinates of line segments)
  * @return A value that is convertible to bool and shows the order of two
  * coordinates
  */
  static int compareEnd(std::pair< cv::Vec4i, float > one,
                        std::pair< cv::Vec4i, float > two) {
    return (one.first[2] > two.first[2]);
  }

  /**
  * @brief Comparison function for std::sort
  * @detail Compares the slopes of the line segments
  * @param one, two pairs of a vector with 4 parameters and a slope
  * @return A value that is convertible to bool and shows the order of two
  * coordinates
  */
  static int compareSlope(std::pair< cv::Vec4i, float > one,
                          std::pair< cv::Vec4i, float > two) {
    return (one.second < two.second);
  }

  /**
  * @brief Finds the absolute difference between two integers
  * @param y1, y2 integers to find the difference between
  * @return Difference between two numbers
  */
  int compareY(int y1, int y2) {
    if (y1 > y2) {
      return (y1 - y2);
    } else {
      return (y2 - y1);
    }
  }

  /**
  * @brief Calculates the slope of the line segment
  * @param one line segment which slope is found
  * @return Slope of the line segment
  */
  float calcSlope(cv::Vec4i one);

  /**
  * @brief Finds the average line segment
  * @detail All the similar lines (please see the strategy in line detection
  * package)
  * are gathered in the vector, then the critical points are taken for the
  * endpoints
  * of average line segment that is generated
  * @param vec a vector of pairs of line segment and slope
  * @return Slope of the line segment
  */
  cv::Vec4i getAverLine(std::vector< std::pair< cv::Vec4i, float > > vec);

  /**
  * @brief Finds the difference(not absolute one) between two integers
  * @param a, b integers to find the difference between
  * @return Difference between two numbers
  */
  int getDifference(int a, int b);

  /**
  * @brief Print lines from a vector to the image (cv::Mat)
  * @param dst, lines  destination image, lines to be mapped to the image
  */
  void printLinesImage(cv::Mat dst, std::vector< cv::Vec4i > lines);

  /**
  * @brief Remove unnecessary lines from a vector of line segments (cv::Mat)
  * @detail As laser scan has noise we detect for each wall a lot of unnecessary
  * lines
  * we eliminate the lines that represent the same wall by finding the slope
  * along with computing the location of endpoints with respect to each other
  * @param lines1 a vector of line segments taken for processing after applying
  * HoughLinesP function from OpenCV
  */
  void removeLines(std::vector< cv::Vec4i > lines1);
  /**
  * @brief Converts the laserScan-data from polar into cartesian coordinates.
  * Then cleans the data from various problems and finally translates the points
  * into pixels on an actual image (in form of an OpenCV-matrix).
  */
  cv::Mat
  createOpenCVImageFromLaserScan(const sensor_msgs::LaserScan::ConstPtr &);

  /** @brief Interpolates the data up to the requested resolution using linear
  * interpolation.
  */
  float interpolate(int, int, std::vector< float >);

  /** @brief Gets number of line segments after elimination
  */
  int getNumLines() { return res.size(); }

  /** @brief Gets last laser scan message
  */
  sensor_msgs::LaserScan getLaserScan() const { return lastScan; }

  /** @brief Sets the image which represents the last scan message received
  *   @param src image to be set
  */
  void setImage(cv::Mat src) { this->src = src; }

  /** @brief Gets the image which represents the last scan message received
  *   @return the image
  */
  cv::Mat getImage() { return src; }

  /** @brief Clears the vector of line segments in order to process the new
  * image
  * from laser scan
  */
  void clearData() { res.clear(); }

  /** @brief Sets the flag for correcting the maneuvre of the robot
  */
  void setCorrecting(bool set) { correcting = set; }

  /** @brief Gets the flag for maneuvre correction of the robot
  */
  bool getCorrecting() { return correcting; }

  /** @brief Gets the vector of line segments that were processed after line
   * removal
  */
  std::vector< cv::Vec4i > getLines() { return res; }

  /** @brief Finds the minimum distance in the range of laser scan message
  *   @param left, right left and right boundaries
  *   @return Returns the slope and the distance to the line segment
  */
  std::pair< float, float > findMinimDistance(int left, int right);

  /** @brief Implements the main logic of the wall-following strategy
  *   @return Returns the twist message containing the next move to be sent
  *   to cmd_vel topic
  */
  const geometry_msgs::Twist controlMovement();

  /** @brief Gets the angle of the robot in the global frame
  */
  float getCurrentAngle() { return robotAngle; }

  /** @brief Sets the angle of the robot in the global frame
 */
  void setCurrentAngle(float angle) { robotAngle = angle; }
};
#endif