#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

int compareStart(cv::Vec4i one, cv::Vec4i two) { return (one[0] < two[0]); }

int compareEnd(std::pair<cv::Vec4i, float> one,
               std::pair<cv::Vec4i, float> two) {
  return (one.first[2] > two.first[2]);
}

int compareSlope(std::pair<cv::Vec4i, float> one,
                 std::pair<cv::Vec4i, float> two) {
  return (one.second < two.second);
}

float calcSlope(cv::Vec4i one) {
  return ((one[3] - one[1]) * 1.0 / (one[2] - one[0]));
}

cv::Vec4i getAverSlope(std::vector<std::pair<cv::Vec4i, float>> vec) {
  // std::sort(vec.begin(), vec.end(), compareSlope);
  // int idx = vec.size()/2;
  cv::Vec4i average;
  average[0] = vec[0].first[0];
  average[1] = vec[0].first[1];

  std::sort(vec.begin(), vec.end(), compareEnd);

  average[2] = vec[0].first[2];
  average[3] = vec[0].first[3];

  // std::cout << average[0] << " , " << average[1] << " , " << average[2] << "
  // , "<< average[3] << std::endl;

  return average;
}

int getDifference(int a, int b) { return abs(a - b); }

int main() {
  cv::Mat src = cv::imread("Image2.jpg");
  if (!src.data)
    return -1;

  cv::Mat bw, color_dst1, color_dst2;
  Canny(src, bw, 50, 200, 3);
  cvtColor(bw, color_dst1, CV_GRAY2BGR);
  cvtColor(bw, color_dst2, CV_GRAY2BGR);
  std::vector<cv::Vec4i> lines1;
  std::vector<std::pair<cv::Vec4i, float>> temp;
  std::vector<cv::Vec4i> res;

  cv::HoughLinesP(bw, lines1, 1, CV_PI / 180, 40, 30, 10);

  std::sort(lines1.begin(), lines1.end(), compareStart);

  bool pushed = true;

  int count = 0;
  for (size_t i = 0; i < lines1.size(); i++) {
    float slope = calcSlope(lines1[i]);
    std::cout << slope << std::endl;

    if (temp.empty()) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
      std::cout << "1" << std::endl;
    } else if (slope > 0 && fabs(temp[0].second - slope) < 0.3 &&
               abs(temp[temp.size() - 1].first[0] - lines1[i][0]) <
                   getDifference(temp[temp.size() - 1].first[0],
                                 temp[temp.size() - 1].first[2]) &&
               abs(temp[temp.size() - 1].first[1] - lines1[i][1]) <
                   getDifference(temp[temp.size() - 1].first[1],
                                 temp[temp.size() - 1].first[3])) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
      std::cout << "3 ?? " << fabs(temp[0].second - slope) << std::endl;
    } else if (slope < 0 && fabs(temp[temp.size() - 1].second + slope) < 0.3 &&
               abs(temp[temp.size() - 1].first[0] - lines1[i][0]) <
                   getDifference(temp[temp.size() - 1].first[0],
                                 temp[temp.size() - 1].first[2]) &&
               abs(temp[temp.size() - 1].first[1] - lines1[i][1]) <
                   getDifference(temp[temp.size() - 1].first[1],
                                 temp[temp.size() - 1].first[3])) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
      std::cout << "4 ?? " << fabs(temp[0].second + slope) << std::endl;
    } else if (abs(temp[temp.size() - 1].first[2] - lines1[i][0]) < 2 &&
               abs(temp[temp.size() - 1].first[3] - lines1[i][1]) < 2) {
      std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i], slope);
      temp.push_back(p);
      std::cout << "5 ?? " << fabs(temp[0].second + slope) << std::endl;
    } else {
      res.push_back(getAverSlope(temp));
      temp.clear();
      std::cout << "6" << std::endl;

      for (int j = 0; j < res.size(); j++) {
        if (slope > 0 && fabs(calcSlope(res[j]) - slope) < 2 ||
            slope < 0 && fabs(calcSlope(res[j]) + slope) < 2) {
          std::cout << "HH" << std::endl;
          pushed = false;
          break;
        }
      }

      if (pushed) {
        std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i], slope);
        std::cout << "YES" << std::endl;
        temp.push_back(p);
        pushed = true;
      }

      // std::pair<cv::Vec4i, float> p = std::make_pair(lines1[i],slope);
      // temp.push_back(p);
    }

    if (!temp.empty() && i == lines1.size() - 1) {
      res.push_back(getAverSlope(temp));
      temp.clear();
      std::cout << "7" << std::endl;
    }

    std::cout << "LINES " << lines1[i][0] << " , " << lines1[i][1] << " , "
              << lines1[i][2] << " , " << lines1[i][3] << std::endl;
    // std::cout << variationStart << std::endl;
  }

  for (size_t i = 0; i < res.size(); i++) {
    line(color_dst2, cv::Point(res[i][0], res[i][1]),
         cv::Point(res[i][2], res[i][3]), cv::Scalar(0, 0, 255), 3, 8);
    std::cout << res[i][0] << " , " << res[i][1] << " , " << res[i][2] << " , "
              << res[i][3] << std::endl;
  }

  for (size_t i = 0; i < lines1.size(); i++) {
    line(color_dst1, cv::Point(lines1[i][0], lines1[i][1]),
         cv::Point(lines1[i][2], lines1[i][3]), cv::Scalar(0, 0, 255), 3, 8);
  }

  // std::cout << lines1.size() << std::endl;
  std::cout << "lines1 size " << lines1.size() << std::endl;
  std::cout << res.size() << " lines detected " << std::endl;

  // cv::imshow("src", src);
  cv::imshow("dst1", color_dst1);
  cv::imshow("dst2", color_dst2);
  cv::waitKey();
  return 0;
}