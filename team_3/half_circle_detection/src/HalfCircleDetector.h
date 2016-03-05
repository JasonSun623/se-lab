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

/** provides an interface for detecting half-circles given a
 * sensor_msgs::LaserScan */
class HalfCircleDetector {

public:
  /** processes a sensor_msgs::LaserScan and calls necessary other functions */
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /** takes a LaserScan and returns an OpenCV-image */
  cv::Mat createOpenCVImageFromLaserScan(
      const sensor_msgs::LaserScan::ConstPtr &laserScan);

  /** takes an OpenCV image and returns the position of the half-circle if it
   * finds one and otherwise (-1, -1) */
  geometry_msgs::Pose2D detectHalfCircle(cv::Mat &image);

  /** getter for last processed half-circle pose */
  geometry_msgs::Pose2D getHalfCirclePose();

  /** setter for last processed half-circle pose */
  void setHalfCirclePose(geometry_msgs::Pose2D &pose);

private:
  /** contains all the points drawn onto the last OpenCV-image */
  std::vector<Point> points;

  /** last computed half-circle pose */
  geometry_msgs::Pose2D halfCirclePose;

  /** uses linear interpolation to increase the resolution of the OpenCV-image
   * generated from a LaserScan */
  float interpolate(int index, int resolution, std::vector<float> data,
                    int size);

  /** converts coordinates from image-frame to robot-frame */
  geometry_msgs::Pose2D createPose(int posX, int posY, int robotX, int robotY);
};

#endif
