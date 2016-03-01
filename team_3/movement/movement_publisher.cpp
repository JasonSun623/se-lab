#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movement_publisher");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

  ros::Rate rate(10);

  while( ros::ok() )
  {
    geometry_msgs::Twist msg;

    msg.linear.x = 4;
    msg.angular.z = 5;

    pub.publish(msg);

    rate.sleep();
  }

  return 0;
}
