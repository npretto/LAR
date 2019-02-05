#pragma once
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "./main.h"
#include "./utils.h"

class DigitOCR {
 private:
  tesseract::TessBaseAPI *ocr;

  void toBlackMask(cv::Mat input, cv::Mat &output);

 public:
  DigitOCR() {}

  void init();

  ~DigitOCR() {
    ocr->End();  // destroy the ocr object (release resources)
  }

  char parse(cv::Mat image);
};
