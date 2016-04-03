/** @file LaserScanAggregation.h
    @brief Takes laserScans and aggregates the contained data for multiple LaserScans.

    @author Felix Schmoll (LiftnLearn)
*/

/* -- Includes -- */

/* ROS include */
#include <ros/ros.h>

/* ROS message type includes */
#include <sensor_msgs/LaserScan.h>

/* OpenCV includes */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* container includes */
#include <vector>

class LaserScanAggregator {};
