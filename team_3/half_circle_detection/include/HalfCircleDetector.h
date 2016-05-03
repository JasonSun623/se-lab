/**
 * @file HalfCircleDetector.h
 * @brief Convert laser scans to circle positions.
 *
 * Provides an interface for detecting half-circles given a
 * sensor_msgs::LaserScan.
 *
 * Returns a pose describing the angle and distance of the detected circle
 * relative to the robot.
 *
 * @author Felix Schmoll (LiftnLearn)
 * @author Leonhard Kuboschek (kuboschek)
 */

#ifndef HALFCIRCLEDETECTOR_H
#define HALFCIRCLEDETECTOR_H

/* -- Includes -- */

/* ROS include */
#include <ros/ros.h>
#include <ros/package.h>

/* ROS message type includes */
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>

/* OpenCV includes */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* container includes */
#include <vector>

/* libs includes */
#include <cmath>

/**
 * @brief Limiting integers to be within a certain range.
 */
#define RANGE(l, x, r) (std::max((l), std::min((r), (x))))

/**
 * @brief Compound class for detecting half-circles.
 */
class HalfCircleDetector {
public:
  /**
   * @brief Constructor for HalfCircleDetector.
   */
  HalfCircleDetector(float laserRange, float mimimumDistance, int stretchFactor,
                     float halfCircleRadius, float minCirclePercentage,
                     float maxInlierDist, float maxCircleDensity,
                     int maxCirclePoints);

  /**
   * @brief Processes a sensor_msgs::LaserScan and calls necessary other
   * functions.
   *
   * The actual magic happens in the called functions.
   * This is just the glue combining everything.
   *
   * @param laserScan Laser scan message that detection is run on.
   * @return Void.
   */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /**
   * @brief Returns last processed half-circle pose
   * @return Last computed half-circle position.
   */
  geometry_msgs::Pose2D getHalfCirclePose();

  /**
   * @brief Returns image from last processed LaserScan.
   * @return Latest OpenCV-image.
   */
  cv::Mat getLaserScanImage() { return laserScanImage; };

private:
  float laserRange;       ///< Maximum measurement distance of laserScanner.
  float minimumDistance;  ///< Prevent taking corners as half-circles if robot
                          ///too close.
  float halfCircleRadius; ///< Approximate radius of half-circle in meters.
  int stretchFactor; ///< Factor to scale data to image (e.g. 100 px for 1m).
  float minCirclePercentage; ///< Cutoff threshold for considering something as
                             ///half circle.
  float maxInlierDist;    ///< Maximum distance of point to object to count as
                          ///circle inlier.
  float maxCircleDensity; ///< Maximum density of points for half-circle.
  int maxCirclePoints;    ///< Maximum number of data points on half-circle.

  cv::Mat laserScanImage; ///< OpenCV representation of latest laserScan.
  geometry_msgs::Pose2D halfCirclePose; ///< Last computed half-circle pose.
  std::vector< cv::Point2f >
      points; ///< Contains points drawn onto the last OpenCV-image.

  /**
   * @brief Takes a LaserScan and returns an OpenCV-image
   *
   * Converts the laserScan-data from polar into cartesian coordinates.
   * Then cleans the data from various problems and finally translates the
   * points into pixels on an actual image (in form of an OpenCV-matrix).
   *
   * @param laserScan Laser scan message that detection is run on.
   * @return Generated OpenCV image.
   */
  cv::Mat createOpenCVImageFromLaserScan(
      const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /**
   * @brief Takes an OpenCV image and returns the position of the half-circle
   * if it finds one and otherwise (-1, -1).
   *
   * Computes possible circles and checks for inlier-percentage. Returns a
   * pose initialised to -1 if no circle is detected.
   * Partially taken from http://stackoverflow.com/a/26234137.
   *
   * @param image An image generated from plotting laser scan points on
   * xy-plane.
   * @return A pose describing half circle position and angle relative to the
   * robot. */
  geometry_msgs::Pose2D detectHalfCircle(cv::Mat &image);

  /**
   * @brief Converts coordinates from image-frame to robot-frame.
   *
   * Basically converts coordinates from cartesian in the image frame
   * to polar in the world frame.
   *
   * @param posX x-coordinate of the object
   * @param posY y-coordinate of the object
   * @param robotX x-coordinate of the robot
   * @param robotY y-coordinate of the robot
   * @return The pose of the given point within the robot frame.
   */
  geometry_msgs::Pose2D createPose(int posX, int posY, int robotX, int robotY);

  /**
   * @brief Computes how much of the circle given is actually present in the
   * picture.
   *
   * Takes a matrix of distances and certifies how much of the given circle if
   * represented.
   *
   * @param dt Matrix of distances to the nearest pixel. Can be created using
   * distance transform.
   * @param[in] center Center of the circle to be verified.
   * @param[in] radius Radius of the circle to be verified.
   * @param[in] semiCircleStart Angle in radius from which semiCircle is
   *searched
   * for (starting at bottom and going clockwise).
   * @return Percentage of circle covered [0,1]
   *
   * @see http://stackoverflow.com/a/26234137
   */
  float verifyCircle(cv::Mat dt, cv::Point2f center, float radius,
                     float semiCircleStart);

  /**
   * @brief Constructs a circle out of three given points on the circle.
   * @param[in] p1,p2,p3 Points on circle
   * @param[out] center Center of circle constructed from given points.
   * @param [out] radius Radius of circle constructed from given points.
   * @return Void.
   * @see http://stackoverflow.com/a/26234137
   */
  void getCircle(cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &p3,
                 cv::Point2f &center, float &radius);

  /**
   * @brief Computes possible candidates for being on the circle.
   * @param[out] first,second,third Points to sample circle from.
   * @param[in] rightIndex,leftIndex Indexes of last rightmost/leftmost points.
   * @param[in] v Points on OpenCV image.
   * @return Void.
   */
  void getSamplePoints(int &first, int &second, int &third, int &rightIndex,
                       int &leftIndex, std::vector< cv::Point2f > &v);

  /**
   * @brief Checks if the point density of the detected semiCircle.
   *
   * A high laserScan-point density might hint towards a false positive
   * (i.e. it is actually a corner).
   *
   * @param[in] boundaries Left and right index of laserScan points circle is
   *constructed from.
   * @return True if it is a real semi-circle.
   */
  bool verifyCircleDensity(std::pair< int, int > &boundaries);

  /**
   * @brief Sets last processed half-circle pose
   * @param pose New half-circle pose
   * @return Void.
   */
  void setHalfCirclePose(geometry_msgs::Pose2D &pose);

  /**
   * @brief Sets image created from last LaserScan.
   * @param image Latest image
   * @return Void.
   */
  void setLaserScanImage(cv::Mat image) { laserScanImage = image; };

  /**
   * @brief Ouputs an image indicating the best guess for a half circle.
   * @return Void.
   */
  void drawHalfCircle(cv::Mat image, float bestCircleRadius,
                      cv::Point2f bestCircleCenter);
};
#endif
