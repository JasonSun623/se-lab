#include <ros/ros.h>
#include "HalfCircleDetector.h"
#define RANGE(l, x, r) (std::max((l), std::min((r), (x))))


void HalfCircleDetector::receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{
  int middle = (int) ((laserScan->angle_max-laserScan->angle_min)/laserScan->angle_increment)/2;

//  ROS_INFO("LaserScan received: [%f]", laserScan->ranges[middle]); 

  cv::Mat image = HalfCircleDetector::createOpenCVImageFromLaserScan(laserScan);
}


cv::Mat HalfCircleDetector::createOpenCVImageFromLaserScan(const sensor_msgs::LaserScan::ConstPtr& laserScan)
{

  int numOfValues = (laserScan->angle_max - laserScan->angle_min) / laserScan->angle_increment;

  int imageHeight = 400;
  int imageWidth = 800;

  cv::Mat image(imageHeight, imageWidth, CV_8UC3, cv::Scalar::all(0));

  for(int i = 0; i < numOfValues; ++i)
  {
    float hyp = laserScan->ranges[i]; 
    float alpha = laserScan->angle_min + i * laserScan->angle_increment;
    int sign = alpha < 0 ? -1 : 1; 

    float opp = std::abs(hyp * std::sin(alpha));
    float adj = hyp * std::cos(alpha);

    //make sure that values are always within bounds
    int x = RANGE(0, (int) ((imageWidth/2) + 100*opp*sign), imageWidth);
    int y = RANGE(0, (int) ((imageHeight/2) + adj*100), imageHeight);

    //ROS_INFO("%d %d %f %f %f", x, y, alpha, laserScan->angle_min, laserScan->angle_max);

    image.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(200, 200, 200);
    //ROS_INFO("laserScan->ranges[%d] = %f", i, laserScan->ranges[i]); 
  } 

  cv::imwrite("/home/robotics/image.jpg", image);

  return image;
}
