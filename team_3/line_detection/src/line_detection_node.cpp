/** @file LineDetector.cpp
  * Implementation of LineDetector.h
  * @author Mariia Gladkova
  */
 
 #include "../include/LineDetector.h"
 #include <ros/package.h>
 
 int main(int argc, char **argv) {
   ros::init(argc, argv, "line_publisher");
   ros::NodeHandle n;
 
   // smart pointer used
   std::unique_ptr< LineDetector > detector(new LineDetector());
   ros::Subscriber sub =
       n.subscribe("base_scan", 1, &LineDetector::receiveLaserScan, &*detector);
 
   cv::Mat bw, color_dst;
   std::vector< cv::Vec4i > lines1;
 
   ros::spinOnce();
   ros::Rate loop_rate(10);
   while (ros::ok()) {
     std::string path = ros::package::getPath("wall_following_strategy/src/");
     cv::Mat src = cv::imread(path + "Image.jpg");
 
     if (!src.data) {
       return -1;
     }
 
     cv::Canny(src, bw, 50, 200, 3);
     cvtColor(bw, color_dst, CV_GRAY2BGR);
     cv::HoughLinesP(bw, lines1, 1, CV_PI / 180, 20, 10, 10);
     // after applying Hough Transform remove unnecessary lines
     detector->removeLines(lines1);
     cv::imwrite(path + "Processed.jpg", color_dst);
     cv::waitKey(25);
 
    std::cout << detector->getNumLines() << " lines detected " << std::endl;
     lines1.clear();
     detector->clearData();
     ros::spinOnce();
     loop_rate.sleep();
   }

   return 0;
 }