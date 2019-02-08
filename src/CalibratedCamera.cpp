
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "main.h"

using namespace cv;
using namespace std;

class CalibratedCamera {
 public:
  bool calibrated = false;
  cv::Mat intrinsic_matrix, distortion_coeffs;
  cv::Mat map1, map2;

  CalibratedCamera() { cout << "CalibratedCamera::ctor()" << endl; }

  void createMaps(cv::Size image_size) {
    cv::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, cv::Mat(),
                                intrinsic_matrix, image_size, CV_16SC2, map1,
                                map2);
  }

  void calibrateFromFolder(std::string path, int board_w = 6, int board_h = 9) {
    int board_n = board_w * board_h;  // number of x (corners) in the chessboard
    cv::Size board_sz = cvSize(board_w, board_h);  // size of checkboard

    vector<vector<cv::Point2f>> image_points;
    vector<vector<cv::Point3f>> object_points;

    namedWindow("calibration", WINDOW_AUTOSIZE);

    cv::Size image_size;

    vector<String> fn;  // std::string in opencv2.4, but cv::String in 3.0
    cv::glob(path, fn, false);  // read file names inside the folder
    int goodImages = 0;
    for (int i = 0; i < fn.size(); i++) {
      auto file = fn.at(i);
      cout << "FILE: " << file << " ";

      Mat img = imread(file, cv::IMREAD_GRAYSCALE);
      image_size = img.size();

      if (img.empty()) {
        cout << "Problema con un file, skippato";
        continue;
      }

      if (DEBUG) imshow("calibration", img);

      vector<cv::Point2f> corners;
      bool found = cv::findChessboardCorners(
          img, board_sz, corners,
          cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

      // Draw it
      drawChessboardCorners(img, board_sz, corners, found);

      if (found) {
        goodImages++;
        image_points.push_back(corners);
        object_points.push_back(vector<cv::Point3f>());
        vector<cv::Point3f> &opts = object_points.back();

        opts.resize(board_n);
        for (int j = 0; j < board_n; j++) {
          opts[j] = cv::Point3f(static_cast<float>(j / board_w),
                                static_cast<float>(j % board_w), 0.0f);
        }
      }
    }

    cout << "Buone immagini trovate:" << goodImages << " su " << fn.size()
         << endl;

    double err = cv::calibrateCamera(
        object_points, image_points, image_size, intrinsic_matrix,
        distortion_coeffs, cv::noArray(), cv::noArray(),
        cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);

    destroyWindow("calibration");

    createMaps(image_size);

    calibrated = true;
    return;
  }

  bool saveCalibrationToFile(String fileName = "./calibration.xml") {
    if (!calibrated) {
      cout << "Camera not calibrated, can't save" << endl;
      return false;
    }
    cv::FileStorage file(fileName, true);
    file << "intrinsic_matrix" << intrinsic_matrix;
    file << "distortion_coeffs" << distortion_coeffs;
    file << "map1" << map1;
    file << "map2" << map2;
    file.release();
    return true;
  }

  bool loadCalibrationFromFile(String fileName = "./calibration.xml") {
    cv::FileStorage file(fileName, cv::FileStorage::READ);
    file["intrinsic_matrix"] >> intrinsic_matrix;
    file["distortion_coeffs"] >> distortion_coeffs;
    file["map1"] >> map1;
    file["map2"] >> map2;
    file.release();
    calibrated = true;
    return true;
  }

  bool undistort(cv::Mat inputImage, cv::Mat &outputImage) {
    if (!calibrated) {
      cout << "The camera is not calibrated, either calibrate from a settings "
              "file or from a folder of images"
           << endl;
      return false;
    }

    if (inputImage.empty()) {
      cout << "The image is empty, i can't undistort it" << endl;
      return false;
    }

    cv::remap(inputImage, outputImage, map1, map2, cv::INTER_LINEAR,
              cv::BORDER_CONSTANT, cv::Scalar());

    return true;
  }
};