#pragma once
#include <iostream>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
//#include "./dubins/dubins.c"
#include "./main.h"

using namespace std;
using namespace cv;

namespace u {

void dilate(const cv::Mat &image, int size = 5);

void erode(const cv::Mat &image, int size = 5);

void blur(const cv::Mat &image, int size = 5, int value = 5);

void dilateErode(const cv::Mat &image, int size = 5);

}  // namespace u
int printConfiguration(double q[3], double total, void *user_data);

vector<Point3f> getDubinPath(const Point3f &a, const Point3f &b);

float point2angle(Point p);

void filter(const cv::Mat &input,cv::Mat &output, string name);


