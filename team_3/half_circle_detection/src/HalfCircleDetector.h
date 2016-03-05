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

class HalfCircleDetector {

public:
  void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);
  cv::Mat createOpenCVImageFromLaserScan(
      const sensor_msgs::LaserScan::ConstPtr &laserScan);
  geometry_msgs::Pose2D detectHalfCircle(cv::Mat &image);

  geometry_msgs::Pose2D getHalfCirclePose();
  void setHalfCirclePose(geometry_msgs::Pose2D &pose);

private:
  std::vector<Point> points;
  geometry_msgs::Pose2D halfCirclePose;

  float interpolate(int index, int resolution, std::vector<float> data,
                    int size);
  geometry_msgs::Pose2D createPose(int posX, int posY, int robotX, int robotY);
};

#endif
