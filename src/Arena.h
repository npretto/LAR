#pragma once
#include <tesseract/baseapi.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "./DigitOCR.h"
#include "./utils.h"

using namespace std;
using namespace cv;

struct sort_atan {
  inline bool operator()(const Point2f &p1, const Point2f &p2) {
    return (atan2(p1.y, p1.x) < atan2(p2.y, p2.x));
  }
};

struct POI {
  Point3f position;
  char c;
};

class Arena {
 private:
  DigitOCR ocr;

  cv::Scalar getColor();

 public:
  static const int width = 500;
  static const int height = 750;
  cv::Mat topView;  // clean top-view, as in input
  // cv::Mat topViewAt16cm;  // at the robot level

  cv::Mat topView_hsv;

  cv::Mat topViewAnnotated;  // top-view with stuff on it

  std::vector<std::vector<cv::Point>> obstacles;
  std::vector<cv::Point> goal;
  std::vector<POI> POIs;

  float getWidth();
  float getHeight();

  bool isPointInside(float x, float y, float buffer = 0);

  bool parseImage(cv::Mat input, bool display = false);

  static bool comparePOI(const POI &p1, const POI &p2);

  void findPOIs(bool display = false);

  void findObstacles(bool display = false);

  void findGoal(bool display = false);

  bool findRobot(cv::Mat const &img, std::vector<double> &state);

  bool getTopViewAt16cm(cv::Mat input, cv::Mat &output,
                        bool debugView = false);

  void getTopView(cv::Mat input, bool debugView = false);

  void drawMapOn(cv::Mat &image);
};
