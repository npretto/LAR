
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "./src/CalibratedCamera.cpp"

using namespace cv;
using namespace std;

const String CALIBRATION_PATH = "./data/calibration";
const bool calibrate = false;

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

  namedWindow("Undistorted", WINDOW_AUTOSIZE);

  vector<String> fn;                     // std::string in opencv2.4, but cv::String in 3.0
  cv::glob(CALIBRATION_PATH, fn, false); //read file names inside the folder
  cout << "folder letta" << endl;
  for (int i = 0; i < fn.size(); i++)
  {
    auto file = fn.at(i);
    cout << "Undistorting FILE: " << file << endl;
    cv::Mat image = cv::imread(file);
    cv::Mat undistorted;
    camera.undistort(image, undistorted);

    Mat sidebyside;
    hconcat(image, undistorted, sidebyside);

    imshow("Display side by side", sidebyside);
    cv::imshow("Undistorted", undistorted);

    waitKey(0);
  }

  waitKey(0);

  // Mat img = imread("images1280/1.jpg", -1);

  // if (img.empty())
  //   return -1;

  // namedWindow("Example 2-2", cv::WINDOW_AUTOSIZE);

  // cv::Size board_sz = cvSize(6, 9);
  // vector<cv::Point2f> corners;
  // bool found = cv::findChessboardCorners(img, board_sz, corners);

  // // Draw it
  // //
  // drawChessboardCorners(img, board_sz, corners, found);

  // imshow("Example 2-2", img);

  // waitKey(0);

  // destroyWindow("Example 2-2");
  return 0;
}
