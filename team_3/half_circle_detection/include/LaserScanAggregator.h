/** @file LaserScanAggregator.h
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


/** @brief Threshold in meters from which on distance values are not considered objects
  * anymore. */
#define LASER_RANGE 3.9

/** @brief Factor by which the distances are scaled up to the image. (i.e. 100 px for
  * 1m)  */
#define STRETCH_FACTOR 100

/** @brief Operating size of container for LaserScans. */
#define AGGREGATOR_SIZE 10

/** @brief Limiting integers to be within a certain range. */
#define RANGE(l, x, r) (std::max((l), std::min((r), (x))))


/** @brief Class that aggregates LaserScans for the purpose of error reduction. */
class LaserScanAggregator {
  public:
    /** @brief Callback for receiving LaserScans.
        @param laserScan LaserScan to be received.
        @return Void. */
    void receiveLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScan);

    /** @brief Takes a LaserScan and translates points from robot-frame to world-frame with robot at (0,0) and the distances being in meters.
      * @param laserScan LaserScan that points are extracted from.
      * @param points Reference to container where points are going to be stored.
      * @return Void. */
    void aggregatePoints(sensor_msgs::LaserScan& laserScan, std::vector<cv::Point2f>& points);

    /** @brief Returns an OpenCV-image built from LaserScanAggregator::laserScans.
      * Converts the laserScan-data from polar into cartesian coordinates.
      * Then cleans the data from various problems and finally translates the
      * points into pixels on an actual image (in form of an OpenCV-matrix).
      * @param imagePoints Aggregate container containing all drawn points in pixel coordinates.
      * @return Generated OpenCV image. */   
    cv::Mat createOpenCVImage(std::vector<cv::Point2f>& imagePoints);

  private:
    /** @brief Aggregate container for received LaserScans. */
    std::vector<sensor_msgs::LaserScan> laserScans; 

};
