/** @file WallFollowingStrategy.cpp
  * Implementation of WallFollowingStrategy.h
  * @author Mariia Gladkova
  * @author Felix Schmoll
  */
#include "../include/WallFollowingStrategy.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_following_strategy");
  ros::NodeHandle n;
  WallFollowingStrategy *strategy = new WallFollowingStrategy();

  // gets laser scan
  ros::Subscriber laserSub = n.subscribe(
      "base_scan", 1, &WallFollowingStrategy::receiveLaserScan, strategy);
  // gets circle position
  ros::Subscriber circleSub =
      n.subscribe("half_circle_detection", 1,
                  &WallFollowingStrategy::receiveCirclePosition, strategy);
  // gets corner handler message
  ros::Subscriber cornerSub =
      n.subscribe("corner_handling", 1,
                  &WallFollowingStrategy::getCornerRecovery, strategy);

  ros::Publisher pub = n.advertise< geometry_msgs::Twist >("cmd_vel", 1);

  ros::Rate rate(10);

  cv::Mat bw, s;
  std::vector< cv::Vec4i > lines1;
  ros::spinOnce();

  strategy->setCurrentAngle(0);

  while (ros::ok()) {
    geometry_msgs::Twist msg;
    if (!strategy->getImage().data) {
      std::string path = ros::package::getPath("wall_following_strategy");
      path += "/src/Image.jpg";
      s = cv::imread(path);
    } else {
      s = strategy->getImage();
    }

    // apply line detection algorithm from the OpenCV library
    // and process the vector
    cv::Canny(s, bw, 50, 200, 3);
    cv::HoughLinesP(bw, lines1, 1, CV_PI / 180, 20, 10, 10);
    strategy->removeLines(lines1);

    msg = strategy->controlMovement();

    pub.publish(msg);

    // clear the data for new laser scan message
    lines1.clear();
    strategy->clearData();

    ros::spinOnce();
    rate.sleep();
  }

  delete strategy;
  return 0;
}