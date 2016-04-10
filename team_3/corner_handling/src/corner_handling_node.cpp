/** @file corner_handling_node.cpp
  *
  * Contains the actual logic of the node (e.g. the ros-loop).
  *
  * @author Felix Schmoll (LiftnLearn)
  */

#include "../include/CornerHandler.h"

/** @brief Main function for node.
  */
int main(int argc, char **argv) {

  ros::init(argc, argv, "corner_handling");

  ros::NodeHandle n;

  CornerHandler cornerHandler;

  ros::Subscriber sub = n.subscribe(
      "base_scan", 1, &CornerHandler::receiveLaserScan, &cornerHandler);

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("corner_handling", 1);

  ros::Rate rate(10);

  while (ros::ok()) {

    geometry_msgs::Twist msg;
    msg = cornerHandler.getTwist();

    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
