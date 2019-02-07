// color_space_hsv.cpp
// Adapted from OpenCV sample:
// samples/cpp/tutorial_code/ImgProc/Threshold_inRange.cpp

#include <stdarg.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

/** Function Headers */
void on_low_h_thresh_trackbar(int, void *);
void on_high_h_thresh_trackbar(int, void *);
void on_low_s_thresh_trackbar(int, void *);
void on_high_s_thresh_trackbar(int, void *);
void on_low_v_thresh_trackbar(int, void *);
void on_high_v_thresh_trackbar(int, void *);

int low_h = 30, low_s = 30, low_v = 30;
int high_h = 100, high_s = 100, high_v = 100;

void printStuff(string name) {
  auto low = cv::Scalar(low_h, low_s, low_v);
  auto high = cv::Scalar(high_h, high_s, high_v);
  FileStorage fs(std::string("./filters/") + name + ".yml", FileStorage::WRITE);

  fs << "filter";  // text - mapping
  fs << "{"
     << "low" << low;
  fs << "high" << high << "}";

  fs.release();
}

int main(int argc, char *argv[]) {
  string filterName = "filter-tool";

  if (argc == 2) {
    cout << argv[0] << " | " << argv[1] << endl;
    filterName = argv[1];
  }

  FileStorage fs;
  fs.open(std::string("./filters/") + filterName + ".yml", FileStorage::READ);

  cv::Scalar low;
  cv::Scalar high;
  // fs["iterationNr"] >> itNr;
  auto node = fs["filter"];
  if (!node.isNone()) {
    vector<float> a, b;
    node["low"] >> a;
    node["high"] >> b;

    cout << "> " << a[0] << " " << a[1] << " " << a[2] << endl;
    cout << "> " << b[0] << " " << b[1] << " " << b[2] << endl;

    low[0] = a[0];
    low[1] = a[1];
    low[2] = a[2];

    high[0] = b[0];
    high[1] = b[1];
    high[2] = b[2];

    cout << low << endl;
    cout << high << endl;

    //   return 0;

    low_h = a[0];
    low_s = a[1];
    low_v = a[2];

    high_h = b[0];
    high_s = b[1];
    high_v = b[2];

    cout << low_h << " " << low_s << " " << low_v << endl;
    cout << high_h << " " << high_s << " " << high_v << endl;
  } else {
    cout << "NOPE " << endl;
  }

  Mat frame, frame_threshold;

  namedWindow("Original Image", WINDOW_AUTOSIZE);
  //   namedWindow("Filtered Image", WINDOW_AUTOSIZE);
  namedWindow("Trackbars", WINDOW_AUTOSIZE);

  createTrackbar("1 HUE LOW", "Trackbars", &low_h, 180,
                 on_low_h_thresh_trackbar);
  createTrackbar("2 HUE HIGH", "Trackbars", &high_h, 180,
                 on_high_h_thresh_trackbar);

  createTrackbar("3 SAT LOW", "Trackbars", &low_s, 255,
                 on_low_s_thresh_trackbar);
  createTrackbar("4 SAT HIGH", "Trackbars", &high_s, 255,
                 on_high_s_thresh_trackbar);

  createTrackbar("5 VALUE LOW", "Trackbars", &low_v, 255,
                 on_low_v_thresh_trackbar);
  createTrackbar("6 VALUE HIGH", "Trackbars", &high_v, 255,
                 on_high_v_thresh_trackbar);

  // this seems to be ignored, cool

  vector<cv::String> fn;  // std::string in opencv2.4, but cv::String in 3.0
  cv::glob("./data/map", fn, false);  // read file names inside the folder
  cout << "folder letta" << endl;

  const int TOTAL_H = 700;
  const int TOTAL_W = 1200;
  int w = 3;
  int h = 3;
  int cell_w = TOTAL_W / w;
  int cell_h = TOTAL_H / h;

  Mat tiled = Mat::zeros(Size(TOTAL_W, TOTAL_H), CV_8UC1);

  //! [trackbar]
  while (static_cast<char>(waitKey(1)) != 'q') {
    std::vector<cv::Mat> frames;
    for (int i = 0; i < 6; i++) {
      auto file = fn.at(i);

      auto f = cv::imread(file);

      cvtColor(f, f, cv::COLOR_BGR2HSV);

      inRange(f, Scalar(low_h, low_s, low_v), Scalar(high_h, high_s, high_v),
              frame_threshold);

      int x = i % w;
      int y = i / w;

      //   Rect ROI(m, n, (int)(x / scale), (int)(y / scale));
      Rect ROI(x * cell_w, y * cell_h, cell_w, cell_h);
      Mat temp;
      resize(frame_threshold, temp, Size(ROI.width, ROI.height));
      temp.copyTo(tiled(ROI));

      imshow("Trackbars", tiled);
    }

    cout << "waiting" << endl;
    cvWaitKey(300);
    cout << "waited, about to write" << endl;
    printStuff(filterName);
    cout << "done writing" << endl;
  }
  return 0;
}
//! [low]
/** @function on_low_h_thresh_trackbar */
void on_low_h_thresh_trackbar(int, void *) {
  low_h = min(high_h - 1, low_h);
  setTrackbarPos("1 HUE LOW", "Trackbars", low_h);
}
//! [low]
//! [high]
/** @function on_high_h_thresh_trackbar */
void on_high_h_thresh_trackbar(int, void *) {
  high_h = max(high_h, low_h + 1);
  setTrackbarPos("2 HUE HIGH", "Trackbars", high_h);
}
//![high]
/** @function on_low_s_thresh_trackbar */
void on_low_s_thresh_trackbar(int, void *) {
  low_s = min(high_s - 1, low_s);
  setTrackbarPos("3 SAT LOW", "Trackbars", low_s);
}

/** @function on_high_s_thresh_trackbar */
void on_high_s_thresh_trackbar(int, void *) {
  high_s = max(high_s, low_s + 1);
  setTrackbarPos("4 SAT HIGH", "Trackbars", high_s);
}

/** @function on_low_v_thresh_trackbar */
void on_low_v_thresh_trackbar(int, void *) {
  low_v = min(high_v - 1, low_v);
  setTrackbarPos("5 VALUE LOW", "Trackbars", low_v);
}

/** @function on_high_v_thresh_trackbar */
void on_high_v_thresh_trackbar(int, void *) {
  high_v = max(high_v, low_v + 1);
  setTrackbarPos("6 VALUE HIGH", "Trackbars", high_v);
}
