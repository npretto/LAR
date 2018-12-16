
#include <tesseract/baseapi.h>  //this has to stay on top otherwise the console will be full of errors :(
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "./src/Arena.cpp"
#include "./src/CalibratedCamera.cpp"
#include "./src/PathFinder.cpp"

const String CALIBRATION_PATH = "./data/calibration";

const bool calibrate = false;
bool STOP_AT_EVERY_OCR = false;

float safetyDistance = 5;
float robotRadius = 20;

CalibratedCamera camera;
Arena arena;
PathFinder pf;

static void onMouse(int event, int x, int y, int, void *) {
  vector<GraphNode *> p = pf.testClick(x, y);
  cv::Mat display(arena.topView.rows, arena.topView.cols, CV_8UC3,
                  Scalar(100, 100, 100));
  arena.drawMapOn(display);
  pf.drawMapOn(display);
  pf.drawPath(display, p);
  cv::imshow("Arena parsed", display);

  cout << "." << endl;
}

int main(int argc, char **argv) {
  if (calibrate) {
    camera.calibrateFromFolder(CALIBRATION_PATH);
    camera.saveCalibrationToFile("./calibration.xml");

    cout << "\n\n\nCAMERA CALIBRATED FORM FOLDER\n\n\n";
    camera.saveCalibrationToFile();
    cout << "\n\n\n calibration saved to file\n\n\n";
  } else {
    cout << "\n\n\n loading calibration from file\n\n\n";

    camera.loadCalibrationFromFile();

    cout << "\n\n\n calibration loaded from file\n\n\n";
  }

  vector<String> fn;  // std::string in opencv2.4, but cv::String in 3.0
  cv::glob("./data/map", fn, false);  // read file names inside the folder
  cout << "folder letta" << endl;

  for (int i = 0; i < fn.size(); i++) {
    auto file = fn.at(i);
    cout << "Undistorting FILE: " << file << endl;
    cv::Mat image = cv::imread(file);
    camera.undistort(image, image);
    arena.parseImage(image);

    cv::imshow("Arena", arena.topView);

    // cv::Mat display = arena.topView;
    cv::Mat display(arena.topView.rows, arena.topView.cols, CV_8UC3,
                    Scalar(100, 100, 100));
    arena.drawMapOn(display);

    cv::imshow("Arena parsed", display);

    pf.fromArena(arena);
    pf.drawMapOn(display);

    cv::imshow("Arena parsed", display);

    setMouseCallback("Arena parsed", onMouse, 0);
    // pf.testClick(500, 600);

    waitKey(0);
    for (;;) {
      waitKey(10);
    }
  }

  waitKey(0);
  return 0;
}
