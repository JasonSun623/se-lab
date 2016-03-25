#include "LineDetector.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "line_detector");
  cv::Mat src = cv::imread("Image2.jpg");
  if (!src.data)
    return -1;

  cv::Mat bw, color_dst1, color_dst2;
  Canny(src, bw, 50, 200, 3);
  cvtColor(bw, color_dst1, CV_GRAY2BGR);
  cvtColor(bw, color_dst2, CV_GRAY2BGR);

  std::vector<cv::Vec4i> lines1;

  cv::HoughLinesP(bw, lines1, 1, CV_PI / 180, 40, 30, 10);

  LineDetector detect;

  detect.removeLines(lines1);

  detect.printLinesImage(color_dst1, lines1);
  detect.printLinesImage(color_dst2, detect.getLines());

  // std::cout << lines1.size() << std::endl;
  std::cout << "lines1 size " << lines1.size() << std::endl;
  std::cout << detect.getNumLines() << " lines detected " << std::endl;

  // cv::imshow("src", src);
  cv::imshow("dst1", color_dst1);
  cv::imshow("dst2", color_dst2);
  cv::waitKey();
  return 0;
}