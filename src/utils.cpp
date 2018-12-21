#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>

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

