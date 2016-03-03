#ifndef HALFCIRCLEDETECTOR_H
#define HALFCIRCLEDETECTOR_H 

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class HalfCircleDetector 
{
  public:

    void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr& laserScan);
    cv::Mat createOpenCVImageFromLaserScan(const sensor_msgs::LaserScan::ConstPtr& laserScan);
    bool semiCircleDetected();
    void processLaserScan(sensor_msgs::LaserScan laserScan);

};

#endif
