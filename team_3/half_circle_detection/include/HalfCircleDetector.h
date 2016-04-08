/** @file HalfCircleDetector.h
 *  @brief Convert laser scans to circle positions.
 *
 *  Provides an interface for detecting half-circles given a
 *  sensor_msgs::LaserScan.
 *
 *  Returns a pose describing the angle and distance of the detected circle
 *  relative to the robot.
 *
 * @author Felix Schmoll
 * @author Leonhard Kuboschek
 * @author Jehanzeb Ayaz
 */

#ifndef HALFCIRCLEDETECTOR_H
#define HALFCIRCLEDETECTOR_H

/* -- Includes -- */

/* ROS include */
#include <ros/ros.h>

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
  * @brief Threshold from which on distance values are not considered objects
 * anymore
  */
#define LASER_RANGE 3.9

/**
  * @brief Epsilon for comparing floating point numbers.
  * Currently used only for comparing pixels, has to be 1 at least
  */
#define EPSILON 1.01

/**
  * @brief Factor by which the distances are scaled up to the image. (100 px for
 * 1m)
  */
#define STRETCH_FACTOR 100

/**
  * @brief Limiting integers to be within a certain range.
  */
#define RANGE(l, x, r) (std::max((l), std::min((r), (x))))

/** @brief If robot to close to obstacle the values become unusable. */
#define MIN_WALL_DISTANCE 0.3
/**
  * @brief Compound class for detecting half-circles.
  */
class HalfCircleDetector {
public:
  /** @brief Processes a sensor_msgs::LaserScan and calls necessary other
   * functions.
   *  The actual magic happens in the called functions.
   *  This is just the glue combining everything.
   *  @param laserScan Laser scan message that detection is run on.
   *  @return Void.
   */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /** @brief Returns last processed half-circle pose
   *  @return Last computed half-circle position */
  geometry_msgs::Pose2D getHalfCirclePose();

  /** @brief Sets last processed half-circle pose
   *  @param pose New half-circle pose */
  void setHalfCirclePose(geometry_msgs::Pose2D &pose);

private:
  /** Contains all the points drawn onto the last OpenCV-image */
  std::vector<cv::Point2f> points;

  /** Last computed half-circle pose */
  geometry_msgs::Pose2D halfCirclePose;

  /** @brief Uses linear interpolation to increase the resolution of the
   * OpenCV-image generated from a LaserScan.
   * Interpolates the data up to the requested resolution using linear
   * interpolation.
   * @param index Index of the point to be interpolated
   * @param resolution Resolution of the generated points
   * @param data Vector of all data points
   * @param size Size of the data vector
   * @return Interpolated function value, or -1 if not successful */
  float interpolate(int index, int resolution, std::vector<float> data);

  /** @brief Takes a LaserScan and returns an OpenCV-image
    * Converts the laserScan-data from polar into cartesian coordinates.
    * Then cleans the data from various problems and finally translates the
   * points
    * into pixels on an actual image (in form of an OpenCV-matrix).
    * @param laserScan Laser scan message that detection is run on.
    * @return Generated OpenCV image
    */
  cv::Mat createOpenCVImageFromLaserScan(
      const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /** @brief Takes an OpenCV image and returns the position of the half-circle
   * if it finds one and otherwise (-1, -1).
   * Computes possible circles and checks for inlier-percentage. Returns a
   * pose initialised to -1 if no circle is detected.
   * Partially taken from http://stackoverflow.com/a/26234137.
   * @param image An image generated from plotting laser scan points on
   * xy-plane.
   * @return A pose describing half circle position and angle relative to the
   * robot. */
  geometry_msgs::Pose2D detectHalfCircle(cv::Mat &image);

  /** @brief Converts coordinates from image-frame to robot-frame
     * Basically converts coordinates from cartesian in the image frame
     * to polar in the world frame.
     *  @param posX x-coordinate of the object
     *  @param posY y-coordinate of the object
     *  @param robotX x-coordinate of the robot
     *  @param robotY y-coordinate of the robot
     *  @return The pose of the given point within the robot frame.
     */
  geometry_msgs::Pose2D createPose(int posX, int posY, int robotX, int robotY);

  /** @brief Computes how much of the circle given is actually present in the
    * picture.
    * Mostly taken from http://stackoverflow.com/a/26234137.
    * Takes a matrix of distances and certifies how much of the given circle if
   * represented.
    * @param dt Matrix of distances to the nearest pixel. Can be created using
   * distance transform.
    * @param center Center of the circle to be verified.
    * @param radius Radius of the circle to be verified.
    * @param inlierSet Reference to container for points detected to be within
   * the circle. Can be for further used for verifying that an actual circle is
   * present.
    * @return Percentage of circle covered [0,1]*/
  float verifyCircle(cv::Mat dt, cv::Point2f center, float radius,
                     std::vector<cv::Point2f> &inlierSet, float semiCircleStart);

  /** @brief Constructs a circle out of three given points on the circle.
    * Mostly taken from http://stackoverflow.com/a/26234137.
    * @return void. Changes arguments passed via reference. */
  void getCircle(cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &p3,
                 cv::Point2f &center, float &radius);

  /** @brief Helper function that computes the direct distance between two
     points.
      @return Distance between given points. */
  float distance(cv::Point2f &a, cv::Point2f &b);

  /** @brief Computes possible candidates for being on the circle.
      @return void. Returns indexes via reference. */
  void getSamplePoints(int &first, int &second, int &third, int &rightIndex,
                       int &leftIndex, std::vector<cv::Point2f> &v);
};
#endif
