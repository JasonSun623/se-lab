
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "RandomWalkStrategy.h"

// Subscribe to: base_scan, half_circle_detection
// Publish to: cmd_vel

int main(int argc, char **argv) {
  ros::init(argc, argv, "random_walk_strategy");
  ros::NodeHandle n;
  RandomWalkStrategy *strategy = new RandomWalkStrategy();

  ros::Subscriber laserSub = n.subscribe(
      "base_scan", 1, &RandomWalkStrategy::receiveLaserScan, strategy);
  ros::Subscriber circleSub =
      n.subscribe("half_circle_detection", 1,
                  &RandomWalkStrategy::receiveCirclePosition, strategy);

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::Rate rate(10);

  ROS_INFO("Starting random walk strategy.");

  while (ros::ok()) {
    geometry_msgs::Twist msg;

    msg = strategy->getControlOutput();

    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  delete strategy;

  return 0;
}
