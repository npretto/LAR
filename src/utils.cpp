#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "../main.h"
#include "./dubins/dubins.c"

using namespace std;
using namespace cv;

namespace u {

void dilate(const cv::Mat &image, int size = 5) {
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size((size * 2) + 1, (size * 2) + 1));
  cv::dilate(image, image, kernel);
}
void erode(const cv::Mat &image, int size = 5) {
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size((size * 2) + 1, (size * 2) + 1));
  cv::erode(image, image, kernel);
}
void dilateErode(const cv::Mat &image, int size = 5) {
  dilate(image, size);
  erode(image, size);
}

}  // namespace u
int printConfiguration(double q[3], double total, void *user_data) {
  vector<Point3f> &path = *(static_cast<vector<Point3f> *>(user_data));
  const double x = q[0];
  const double y = q[1];
  const double theta = q[2];

  Point2f a(x, y);

  Point2f b(x + cos(theta) * 5, y + sin(theta) * 5);

  path.push_back(Point3f(x, y, theta));

  line(display, a, b, Scalar(120, 255, 120), 1);
  circle(display, a, 1, Scalar(20, 20, 255), 1, LINE_AA);
  // cv::imshow("Arena parsed", display);

  // cout << "WAIT" << endl;
  // cvWaitKey(30);
  // printf("%f, %f, %f, %f\n", q[0], q[1], q[2], total);
  return 0;
}

vector<Point3f> getDubinPath(const Point3f &a, const Point3f &b) {
  vector<Point3f> path;
  double q0[] = {a.x, a.y, a.z};
  double q1[] = {b.x, b.y, b.z};
  double turning_radius = cmToPixels(15);
  DubinsPath p;
  dubins_shortest_path(&p, q0, q1, turning_radius);
  // cout << "PATH ,,,,,,,,,,,,,,,,,,,," << endl;
  dubins_path_sample_many(&p, 10, printConfiguration, &path);
  return path;
}

float point2angle(Point p) { return atan2(p.y, p.x); }
