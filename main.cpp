
#include <tesseract/baseapi.h> //this has to stay on top otherwise the console will be full of errors :(
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "./src/CalibratedCamera.cpp"
#include "./src/Arena.cpp"
#include "./src/PathFinder.cpp"

const String CALIBRATION_PATH = "./data/calibration";

const bool calibrate = false;
bool STOP_AT_EVERY_OCR = false;

int main(int argc, char **argv)
{

  CalibratedCamera camera;

  if (calibrate)
  {

    camera.calibrateFromFolder(CALIBRATION_PATH);
    camera.saveCalibrationToFile("./calibration.xml");

    cout << "\n\n\nCAMERA CALIBRATED FORM FOLDER\n\n\n";
    camera.saveCalibrationToFile();
    cout << "\n\n\n calibration saved to file\n\n\n";
  }
  else
  {
    cout << "\n\n\n loading calibration from file\n\n\n";

    camera.loadCalibrationFromFile();

    cout << "\n\n\n calibration loaded from file\n\n\n";
  }

  vector<String> fn;                 // std::string in opencv2.4, but cv::String in 3.0
  cv::glob("./data/map", fn, false); //read file names inside the folder
  cout << "folder letta" << endl;

  for (int i = 0; i < fn.size(); i++)
  {
    auto file = fn.at(i);
    cout << "Undistorting FILE: " << file << endl;
    cv::Mat image = cv::imread(file);
    camera.undistort(image, image);
    Arena arena;
    arena.parseImage(image);

    cv::imshow("Arena", arena.topView);

    // cv::Mat display = arena.topView;
    cv::Mat display(arena.topView.rows, arena.topView.cols, CV_8UC3, Scalar(100, 100, 100));
    arena.drawMapOn(display);

    cv::imshow("Arena parsed", display);

    PathFinder pf(arena);

    pf.drawMapOn(display);

    cv::imshow("Arena parsed", display);

    waitKey(0);
  }

  waitKey(0);
  return 0;
}
