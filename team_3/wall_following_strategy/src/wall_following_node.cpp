/** @file WallFollowingStrategy.cpp
  * Implementation of WallFollowingStrategy.h
  * @author Mariia Gladkova
  * @author Felix Schmoll
  */
#include "../include/WallFollowingStrategy.h"

/** @brief Starts the node.
  * @param[in] argc Number of Arguments
  * @param[in] argv Array of Arguments
  *   - [0] Program name
  *   - [1] Topic name for the node
  *   - [2] Topic name for receiving position of detected half circles
  *   - [3] Topic name for receiving movement commands in case of crashing into the obstacle
  *   - [4] Topic name for receiving OpenCV images
  *   - ... other ros-specific arguments
  */

int main(int argc, char **argv) {
  ROS_ASSERT_MSG(argc > 4,
                 "Not enough arguments for topic names. 5 expected, %d given.",
                 argc);

  ros::init(argc, argv, argv[1]);
  ros::NodeHandle node(argv[1]);

  float linearVelocity;
  float wallDistance;
  float crashVelocity;
  float turnCorrection;
  float turnCircleCorrection;

  ROS_ASSERT(node.getParam("LINEAR_VEL", linearVelocity));
  ROS_ASSERT(node.getParam("WALL_FOLLOWING_DISTANCE", wallDistance));
  ROS_ASSERT(node.getParam("CRASH_VELOCITY", crashVelocity));
  ROS_ASSERT(node.getParam("TURN_CORRECTION", turnCorrection));
  ROS_ASSERT(node.getParam("TURN_CIRCLE_CORRECTION", turnCircleCorrection));

  WallFollowingStrategy strategy(linearVelocity, wallDistance, crashVelocity,
                                 turnCorrection, turnCircleCorrection);

  /** Subscribers */
  ros::Subscriber laserSub = node.subscribe(
      "/base_scan", 1, &WallFollowingStrategy::receiveLaserScan, &strategy);
  ros::Subscriber circleSub =
      node.subscribe(argv[2], 1,
                     &WallFollowingStrategy::receiveCirclePosition, &strategy);
  ros::Subscriber crashSub =
      node.subscribe(argv[3], 1,
                     &WallFollowingStrategy::getCrashRecovery, &strategy);

  image_transport::ImageTransport imageTransport(node);
  image_transport::Subscriber imageSub;

  imageSub = imageTransport.subscribe(
      argv[4], 1, &WallFollowingStrategy::receiveOpenCVImage,
      &strategy);

  /** Publishers */
  ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

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
