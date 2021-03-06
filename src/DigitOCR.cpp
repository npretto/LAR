#include "./DigitOCR.h"

void DigitOCR::toBlackMask(cv::Mat input, cv::Mat &output) {
  cv::Mat hsv;
  cv::Mat green;
  cv::Mat black;

  cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);
  // auto meanValue = mean(input) * 2;
  // FILTER FOR BLACK COLOR

  cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 80), black);
  u::dilate(black, 2);
  if (DEBUG) cv::imshow("black", black);
  cv::moveWindow("black", 800, 0);
  // FIND GREEN COLOR
  cv::inRange(hsv, cv::Scalar(130 / 2 - 10, 0, 40),
              cv::Scalar(130 / 2 + 10, 255, 255), green);
  u::erode(green, 3);

  if (DEBUG) cv::imshow("green", green);
  cv::moveWindow("green", 900, 0);

  // REMOVE GREEN PARTS
  output = black - green;

  if (DEBUG) cv::imshow("digit_mask", output);
  cv::moveWindow("digit_mask", 1000, 0);

  u::erode(output, 2);
  u::dilate(output, 2);

  if (DEBUG) cv::imshow("digit_mask_eroded", output);
  cv::moveWindow("digit_mask_eroded", 1100, 0);
}

void DigitOCR::init() {
  ocr = new tesseract::TessBaseAPI();  // Create Tesseract object
  ocr->Init(NULL, "eng");  // Initialize tesseract to use English (eng)
  ocr->SetPageSegMode(
      tesseract::PSM_SINGLE_CHAR);  // Image contains a single character
  // ocr->SetVariable("tessedit_char_whitelist", "01234"); // Only digits are
  // valid output
  ocr->SetVariable("tessedit_char_whitelist",
                   "012345689");  // Only digits are valid output
}

char DigitOCR::parse(cv::Mat image) {
  cv::Mat black_mask;
  toBlackMask(image, black_mask);
  if (DEBUG) cv::imshow("digit", image);
  cv::moveWindow("digit", 600, 0);

  ocr->SetImage(black_mask.data, black_mask.cols, black_mask.rows, 1,
                black_mask.step);

  char *detected;

  std::map<char, int> confidence;

  int i = 0;
  const int step = 5;

  // cout << "\n\n\nOCR\n\n\n";
  while (i <= 360 / step) {
    cv::Mat rotated = black_mask.clone();
    auto r = cv::getRotationMatrix2D(
        cv::Point(rotated.cols / 2, rotated.rows / 2), step * i, 1);
    cv::warpAffine(rotated, rotated, r, rotated.size());

    ocr->SetImage(rotated.data, rotated.cols, rotated.rows, 1, rotated.step);
    detected = ocr->GetUTF8Text();
    int conf = *(ocr->AllWordConfidences());

    int threshold = 30;

    confidence[*detected] += conf > threshold ? conf - threshold : 0;

    // cout << "-----------------" << endl;

    // cout << "DETECTED: ->" << detected << std::endl;

    // cout << "CONFIDENCE " << conf << endl;
    // cout << "TOTAL CONFIDENCE " << confidence[*detected] << endl;

    // if(DEBUG) cv::imshow("digit_mask_eroded", rotated);

    // if(DEBUG) cv::imshow("digit_mask", rotated);

    // cvWaitKey(300);
    i += step;
  }

  // cout << "TOTAL CONF" << endl;

  confidence[' '] = 0;
  confidence['0'] = 0;
  confidence['6'] = 0;
  confidence['7'] = 0;
  confidence['8'] = 0;
  confidence['9'] = 0;

  char c;
  int max = 0;
  for (auto const &x : confidence) {
    if (x.second > max) {
      max = x.second;
      c = x.first;
    }
    if (DEBUG) std::cout << x.first << ':' << x.second << std::endl;
  }

  cout << "DETECTED " << c << endl;

  return c;
}