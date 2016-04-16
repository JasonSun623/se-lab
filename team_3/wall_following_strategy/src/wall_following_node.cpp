/** @file WallFollowingStrategy.cpp
  * Implementation of WallFollowingStrategy.h
  * @author Mariia Gladkova
  * @author Felix Schmoll
  */
#include "../include/WallFollowingStrategy.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_following_strategy");

  ros::NodeHandle n;
  WallFollowingStrategy strategy;

  ros::Subscriber laserSub = n.subscribe(
      "base_scan", 1, &WallFollowingStrategy::receiveLaserScan, &strategy);
  ros::Subscriber circleSub =
      n.subscribe("half_circle_detection", 1,
                  &WallFollowingStrategy::receiveCirclePosition, &strategy);
  ros::Subscriber crashSub = n.subscribe(
      "crash_recovery", 1, &WallFollowingStrategy::getCrashRecovery, &strategy);

  ros::NodeHandle imageHandle;
  image_transport::ImageTransport imageTransport(imageHandle);
  image_transport::Subscriber imageSub;

  imageSub = imageTransport.subscribe(
      "laserScan_image", 1, &WallFollowingStrategy::receiveOpenCVImage,
      &strategy);

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate rate(10);

  cv::Mat bw, s;
  std::vector<cv::Vec4i> lines;
  ros::spinOnce();

  strategy.setCurrentAngle(0);

  while (ros::ok()) {
    geometry_msgs::Twist msg;

    if (strategy.getImage().size().height) {
      s = strategy.getImage();
      cv::Canny(s, bw, 50, 200, 3);
      cv::HoughLinesP(bw, lines, 1, CV_PI / 180, 20, 10, 10);
      strategy.removeLines(lines);
    }

    msg = strategy.controlMovement();

    pub.publish(msg);

    lines.clear();
    strategy.clearData();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
