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
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/** include OpenCV */
#include <opencv2/opencv.hpp>

/** include C library */
#include <assert.h>
#include <vector>
#include <algorithm>
#include <cmath>

/**
  * @brief Upper speed for turning rate
  */
#define MAX_TURN 0.8f

/**
 * @brief Smaller values for slope of the line segment are neglected
 */
#define SLOPE_EPSILON 0.1f

/**
 * @brief Threshold for the number of circles detected in order to start approaching the circle
 */
#define CIRCLE_COUNT 4

/**
 * @brief Wall variation threshold while wall following
 */
#define GLOBAL_WALL_VARIATION 0.05f

/**
 * @brief Bigger distance from the wall is considered as "lost-in-space" state
 */
#define LOST_THRESHOLD 1.0f

/**
 * @brief Compound class for wall-following strategy implementation
 */
class WallFollowingStrategy {
private:
  /** @brief true if the circle is currently visible */
  bool circleVisible;

  /** @brief The angle of the circle relative to the robot (in radian) */
  float circleAngle;

  /** @brief The distance of the circle (in meters) */
  float circleDistance;

  /** @brief RobotAngle */
  float robotAngle;

  /** @brief Counter for deciding circleFoundMode */
  int circleCallCount;

  /** @brief true if CrashRecovery has reported a crash */
  bool crashMode;

  /** @brief true if we're following a wall */
  bool followWall;

  /** @brief true if we're prioritizing following the circle over other maneuvers */
  bool circleFoundMode;

  /** @brief true if the robot is steering to chase the circle */
  bool correcting;

  /** @brief Current ununsed */
  bool stuck;

  /** @brief true if we're freely maneuvering (for example in the beginning) */
  bool start;

  /** @brief Velocity of the robot when moving in a straight line */
  float linearVelocity;

  /** @brief The distance the wall will be followed at */
  float wallDistance;

  /** @brief Used when robot is very close to an obstacle */
  float crashVelocity;

  /** @brief This value is multiplied by the error, in order to turn around walls */
  float turnCorrection;

  /** @brief This value is multiplied by the error, to steer into the circle */
  float turnCircleCorrection;

  bool lostMode;

  /** @brief Resulting line segments */
  std::vector<cv::Vec4i> res;

  /** @brief Records the first line that was chosen */
  std::vector<std::pair<float, float>> initialLineChoice;

  /** @brief Stores the last received laser scan message */
  sensor_msgs::LaserScan lastScan;

  /** @brief Data source for line detection */
  cv::Mat src;

  /** @brief Stores the current crash resolution obtained from CrashRecovery */
  geometry_msgs::Twist crashHandler;

public:
  /** @brief Default constructor for Wall Following Strategy.
   */
  WallFollowingStrategy() {
    circleFoundMode = false;
    start = true;
    lostMode = false;
    circleCallCount = 0;
    this->linearVelocity = 0.3;
    this->wallDistance = 0.3;
    this->crashVelocity = -0.2;
    this->turnCorrection = 0.01;
    this->turnCircleCorrection = 0.035;
  }
  /** @brief Constructor for Wall Following Strategy.
   *  Here the environment variables are loaded.
   */
  WallFollowingStrategy(float linearVelocity, float wallDistance,
                        float crashVelocity, float turnCorrection,
                        float turnCircleCorrection) {
    start = true;
    lostMode = false;
    circleCallCount = 0;
    circleFoundMode = false;
    this->linearVelocity = linearVelocity;
    this->wallDistance = wallDistance;
    this->crashVelocity = crashVelocity;
    this->turnCorrection = turnCorrection;
    this->turnCircleCorrection = turnCircleCorrection;
  }

  /**
  * @brief Gets position of a circle if detected as a Pose2D message
  * @param circlePos Pose2D message about location of the circle(x,y coordinates
  * and angle)
  */
  void receiveCirclePosition(const geometry_msgs::Pose2D::ConstPtr &circlePos);

  /**
  * @brief Receives a laser scan message
  * @param laserScan LaserScan Message with information about the distances
  */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /**
   * @brief Callback for receiving OpenCV images from half_circle_detection.
   * @param msg Pointer to the new OpenCV image
   */
  void receiveOpenCVImage(const sensor_msgs::ImageConstPtr &msg);

  /**
  * @brief Receives a message from crash_recovery topic of robot behavior
  * in case of crashing into the obstacle
  * @details Slowly moves the robot opposite to
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
  * @brief Comparison function for std::sort
  * @details Compares the x-coordinates of starting points of the line segments
  * @param one, two Starting and ending position coordinates of line segments
  * @return A value that is convertible to bool and shows the order of two
  * coordinates
  */
  static int compareStart(cv::Vec4i one, cv::Vec4i two) {
    return (one[0] < two[0]);
  }

  /**
  * @brief Comparison function for std::sort
  * @details Compares the x-coordinates of ending points of the line segments
  * @param one, two Vectors with 4 parameters (Starting and ending position
  * coordinates of line segments)
  * @return A value that is convertible to bool and shows the order of two
  * coordinates
  */
  static int compareEnd(std::pair<cv::Vec4i, float> one,
                        std::pair<cv::Vec4i, float> two) {
    return (one.first[2] > two.first[2]);
  }

  /**
  * @brief Comparison function for std::sort
  * @details Compares the slopes of the line segments
  * @param one, two pairs of a vector with 4 parameters and a slope
  * @return A value that is convertible to bool and shows the order of two
  * coordinates
  */
  static int compareSlope(std::pair<cv::Vec4i, float> one,
                          std::pair<cv::Vec4i, float> two) {
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
  * @details All the similar lines (please see the strategy in line detection
  * package)
  * are gathered in the vector, then the critical points are taken for the
  * endpoints
  * of average line segment that is generated
  * @param vec a vector of pairs of line segment and slope
  * @return Slope of the line segment
  */
  cv::Vec4i getAverLine(std::vector<std::pair<cv::Vec4i, float>> vec);

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
  void printLinesImage(cv::Mat dst, std::vector<cv::Vec4i> lines);

  /**
  * @brief Remove unnecessary lines from a vector of line segments (cv::Mat)
  * @details As laser scan has noise we detect for each wall a lot of
  * unnecessary
  * lines
  * we eliminate the lines that represent the same wall by finding the slope
  * along with computing the location of endpoints with respect to each other
  * @param lines1 a vector of line segments taken for processing after applying
  * HoughLinesP function from OpenCV
  */
  void removeLines(std::vector<cv::Vec4i> lines1);

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

  /**@brief Sets the number of detected circles
   * @param count number to be set
   */
  void setCount(int count) {
    if (count < 0){
      std::cerr << "Incorrect input data" << std::endl;
      return;
    }
    circleCallCount = count;
  }

  /**@brief Gets the number of detected circles
   * @return the number of detected circles
   */
  int getCount() {
    return circleCallCount;
  }

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
  std::vector<cv::Vec4i> getLines() { return res; }

  /** @brief Finds the minimum distance in the range of laser scan message
  *   @param left, right left and right boundaries
  *   @return Returns the slope and the distance to the line segment
  */
  std::pair<float, float> findMinimDistance(int left, int right);

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

  /** @brief Returns whether the condition for elimination of line
   * segments is satisfied
   */
  bool lineCondition(std::pair<cv::Vec4i, float>, cv::Vec4i);

  /** @brief Helper function for retrieving float environment variables declared
   * in the launch file. */
  void getEnvironmentVariable(const char *varString, float &var) {
    char *c;

    c = std::getenv(varString);
    if (!c) {
      ROS_INFO(
          "Environment variable %s not found. Using %lf as a default value.",
          varString, var);
    } else {
      var = std::stof(std::string(c));
    }
  }
  /** @brief Increments the turning angle of the robot [deg]
   */
  void incrementTurn(float);
};
#endif
