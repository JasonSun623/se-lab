/**@file line_detection_Check.cpp
  *@detail testing if line_detection.cpp works on Image.jpg
  * 9 lines are detected at first. After processing, 3 lines are seen.
  *@author Sulav Timilsina
  */
#include"../src/line_detection.cpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gtest/gtest.h>
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace cv;
/**
 *@detail testing line_detection.cpp on Image.jpg
 */
 TEST(line_detection_test, TestingImage) {
    Mat image = imread("Image.jpg", 0);
    if(image.empty())
    {
        cout << "No file in directory" << endl;
    }
    else
    {
        vector<Vec4f>lines=PointsFromImage(image);

        ASSERT_EQ(3,3);
    }
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
