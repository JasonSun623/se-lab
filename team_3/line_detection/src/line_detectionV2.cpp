#include "LineDetector.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "line_publisher");
  ros::NodeHandle n;

  std::unique_ptr<LineDetector> detector(new LineDetector());
  ros::Subscriber sub =
      n.subscribe("base_scan", 1, &LineDetector::receiveLaserScan, &*detector);

  cv::Mat bw, color_dst1, color_dst2;
  std::vector<cv::Vec4i> lines1;
  ros::Rate rate(10);
  while (ros::ok()) {
    if (detector->getLaserScan()) {
      cv::Mat src =
          detector->createOpenCVImageFromLaserScan(detector->getLaserScan());
      if (!src.data)
        return -1;

      Canny(src, bw, 50, 200, 3);
      cvtColor(bw, color_dst1, CV_GRAY2BGR);
      cvtColor(bw, color_dst2, CV_GRAY2BGR);

      cv::HoughLinesP(bw, lines1, 1, CV_PI / 180, 40, 30, 10);

      detector->removeLines(lines1);

      detector->printLinesImage(color_dst1, lines1);
      detector->printLinesImage(color_dst2, detector->getLines());

      // std::cout << lines1.size() << std::endl;
      std::cout << "lines1 size " << lines1.size() << std::endl;
      std::cout << detector->getNumLines() << " lines detected " << std::endl;

      cv::imshow("src", src);
      cv::imshow("dst1", color_dst1);
      cv::imshow("dst2", color_dst2);
    }
    rate.sleep();
  }

  return 0;
}