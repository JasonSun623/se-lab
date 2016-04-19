#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include "../include/CrashRecoverer.h"

/** @brief Helper function for retrieving float environment variables declared
   * in the launch file. */
  void getEnvironmentVariable(const char *varString, char *var) {
    char *c;

    c = std::getenv(varString);
    if (!c) {
      ROS_INFO(
          "Environment variable %s not found. Using %lf as a default value.",
          varString, var);
    } else {
      var = c
    }
  }
};

/** Main executable for crash resolution advice node.
 *
 *  Subscribes to ```base_scan``` for laser data.
 *  Publishes to ```crash_recovery``` for publishing resolution advice. */
int main(int argc, char **argv) {
  std::string laser_topic = "base_scan";
  std::string crash_topic = "crash_recovery";

  getEnvironmentVariable("laser_topic", &laser_topic);
  getEnvironmentVariable("crash_topic", &crash_topic);

  ros::init(argc, argv, crash_topic);

  ros::NodeHandle n;

  CrashRecoverer *recover = new CrashRecoverer();

  ros::Subscriber sub = n.subscribe(
      laser_topic, 1, &CrashRecoverer::receiveLaserScan, recover);

  ros::Publisher pub =
      n.advertise<geometry_msgs::Twist>(crash_topic, 1);

  ros::Rate rate(10);

  while (ros::ok()) {

    pub.publish(recover->getResolution());

    ros::spinOnce();
    rate.sleep();
  }

  delete recover;

  return 0;
}
