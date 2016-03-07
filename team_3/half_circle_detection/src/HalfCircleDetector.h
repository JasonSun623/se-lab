#ifndef HALFCIRCLEDETECTOR_H
#define HALFCIRCLEDETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

typedef std::pair<int, int> Point;

/** @file HalfCircleDetector.h
 *  @brief Convert laser scans to circle positions.
 *
 *  Provides an interface for detecting half-circles given a
 *  sensor_msgs::LaserScan.
 *
 *  Returns a pose describing the angle and distance of the detected circle
 *  relative to the robot.
 * @author Felix Schmoll
 * @author Leonhard Kuboschek
 */
class HalfCircleDetector {

public:
  /** @brief Processes a sensor_msgs::LaserScan and calls necessary other functions
   *  @param laserScan Laser scan message that detection is run on.
   */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /** @brief Takes a LaserScan and returns an OpenCV-image
   *  @param laserScan Laser scan message that detection is run on.
   *  @return Generated OpenCV image
   */
  cv::Mat createOpenCVImageFromLaserScan(
      const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /** @brief Takes an OpenCV image and returns the position of the half-circle if it
   * finds one and otherwise (-1, -1)
   * @param image An image generated from plotting laser scan points on xy-plane.
   * @return A pose describing half circle position and angle relative to the robot. */
  geometry_msgs::Pose2D detectHalfCircle(cv::Mat &image);

  /** @brief Returns last processed half-circle pose
   *  @return Last computed half-circle position */
  geometry_msgs::Pose2D getHalfCirclePose();

  /** @brief Sets last processed half-circle pose
   *  @param pose New half-circle pose */
  void setHalfCirclePose(geometry_msgs::Pose2D &pose);

private:
  /** Contains all the points drawn onto the last OpenCV-image */
  std::vector<Point> points;

  /** Last computed half-circle pose */
  geometry_msgs::Pose2D halfCirclePose;

  /** @brief Uses linear interpolation to increase the resolution of the OpenCV-image
   * generated from a LaserScan
   * @param index Index of the point to be interpolated
   * @param resolution Resolution of the generated points
   * @param data Vector of all data points
   * @param size Size of the data vector
   * @return Interpolated function value */
  float interpolate(int index, int resolution, std::vector<float> data,
                    int size);

  /** Converts coordinates from image-frame to robot-frame */
  geometry_msgs::Pose2D createPose(int posX, int posY, int robotX, int robotY);
};

#endif
