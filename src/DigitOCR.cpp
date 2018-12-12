#pragma once
#include <tesseract/baseapi.h> // Tesseract headers
#include <leptonica/allheaders.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include "./utils.cpp"
#include "../main.h"

class DigitOCR
{
  private:
    tesseract::TessBaseAPI *ocr;

    void toBlackMask(cv::Mat input, cv::Mat &output)
    {
        cv::Mat hsv;
        cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);

        cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 30 * 255 / 100), output);
    }

  public:
    DigitOCR()
    {
        ocr = new tesseract::TessBaseAPI();              // Create Tesseract object
        ocr->Init(NULL, "eng");                          // Initialize tesseract to use English (eng)
        ocr->SetPageSegMode(tesseract::PSM_SINGLE_CHAR); // Image contains a single character
        // ocr->SetVariable("tessedit_char_whitelist", "01234"); // Only digits are valid output
        ocr->SetVariable("tessedit_char_whitelist", "0123456789"); // Only digits are valid output
    }
    char *parse(cv::Mat image)
    {
        cv::Mat black_mask;
        toBlackMask(image, black_mask);

        cv::imshow("digit", image);
        cv::moveWindow("digit", 600, 0);

        cv::imshow("digit_mask", black_mask);
        cv::moveWindow("digit_mask", 700, 0);

        u::erode(black_mask, 2);
        u::dilate(black_mask, 2);

        cv::imshow("digit_mask_eroded", black_mask);
        cv::moveWindow("digit_mask_eroded", 800, 0);

        ocr->SetImage(black_mask.data, black_mask.cols, black_mask.rows, 1, black_mask.step);
        char *detected = ocr->GetUTF8Text();

        // int i = 1;
        // while (*detected == ' ')
        // {
        //     std::cout << "NOTHING DETECTED" << std::endl;
        //     cv::Mat rotated = black_mask.clone();
        //     auto r = cv::getRotationMatrix2D(cv::Point(rotated.cols / 2, rotated.rows / 2), i, 1);
        //     cv::warpAffine(rotated, rotated, r, rotated.size());
        //     cv::imshow("digit_mask", black_mask);

        //     ocr->SetImage(black_mask.data, black_mask.cols, black_mask.rows, 1, black_mask.step);
        //     detected = ocr->GetUTF8Text();

        //     cv::waitKey();
        // }

        std::cout << "DETECTED: " << detected << std::endl;

        return detected;
    }

    ~DigitOCR()
    {
        ocr->End(); // destroy the ocr object (release resources)
    }
};