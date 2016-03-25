#include "LineDetector.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "line_publisher");
  ros::NodeHandle n;

  std::unique_ptr<LineDetector> detector(new LineDetector());
  ros::Subscriber sub =
      n.subscribe("base_scan", 1, &LineDetector::receiveLaserScan, &*detector);

  cv::Mat bw, color_dst;
  std::vector<cv::Vec4i> lines1;

  ros::spinOnce();
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    // Please specify your absolute directory!
    cv::Mat src = cv::imread("/home/mgladkova/copy_ws/src/team-3/team_3/"
                             "line_detection/src/Image.jpg");

    if (!src.data) {
      return -1;
    }

    cv::Canny(src, bw, 50, 200, 3);
    cvtColor(bw, color_dst, CV_GRAY2BGR);

    cv::HoughLinesP(bw, lines1, 1, CV_PI / 180, 20, 10, 10);
    detector->removeLines(lines1);
    detector->printLinesImage(color_dst, detector->getLines());
    // Please specify your absolute directory!
    cv::imwrite("/home/mgladkova/copy_ws/src/team-3/team_3/line_detection/src/"
                "Processed.jpg",
                color_dst);
    cv::waitKey(25);
    std::cout << detector->getNumLines() << " lines detected " << std::endl;
    lines1.clear();
    detector->clearData();
    ros::spinOnce();
    loop_rate.sleep();

    // detector->printLinesImage(color_dst1, lines1);
    // detector->printLinesImage(color_dst2, detector->getLines());

    // // std::cout << lines1.size() << std::endl;
    // std::cout << "lines1 size " << lines1.size() << std::endl;

    // cv::imshow("src", src);
    // cv::imshow("dst1", color_dst1);
    // cv::imshow("dst2", color_dst2);
  }

  return 0;
}