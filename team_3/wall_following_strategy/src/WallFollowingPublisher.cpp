#include "WallFollowingStrategy.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_following_strategy");
  ros::NodeHandle n;
  WallFollowingStrategy *strategy = new WallFollowingStrategy();

  ros::Subscriber laserSub = n.subscribe(
      "base_scan", 1, &WallFollowingStrategy::receiveLaserScan, strategy);
  ros::Subscriber circleSub =
      n.subscribe("half_circle_detection", 1,
                  &WallFollowingStrategy::getCirclePosition, strategy);

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate rate(10);

  cv::Mat bw, color_dst;
  std::vector<cv::Vec4i> lines1;
  ros::spinOnce();

  strategy->setCurrentAngle(0);

  while (ros::ok()) {
    geometry_msgs::Twist msg;
    cv::Mat s = cv::imread("/home/mgladkova/copy_ws/src/team-3/team_3/"
                           "wall_following_strategy/src/Image.jpg");
    cv::Canny(s, bw, 50, 200, 3);
    cvtColor(bw, color_dst, CV_GRAY2BGR);

    cv::HoughLinesP(bw, lines1, 1, CV_PI / 180, 20, 10, 10);
    strategy->removeLines(lines1);

    msg = strategy->controlMovement();

    strategy->printLinesImage(color_dst, strategy->getLines());

    pub.publish(msg);

    cv::imwrite("/home/mgladkova/copy_ws/src/team-3/team_3/"
                "wall_following_strategy/src/Processed.jpg",
                color_dst);
    cv::waitKey(25);

    lines1.clear();
    strategy->clearData();

    ros::spinOnce();
    rate.sleep();
  }

  delete strategy;
  return 0;
}