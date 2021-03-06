#include <tesseract/baseapi.h>  //this has to stay on top otherwise the console will be full of errors :(

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "./src/main.h"

using namespace std;
using namespace cv;

#include "./src/robot_project.h"

int main(int argc, char* argv[]) {
  cout << "MMMMM  " << endl;

  vector<cv::String> fn;  // std::string in opencv2.4, but cv::String in 3.0
  cv::glob("./data/map", fn, false);  // read file names inside the folder
  cout << "folder letta" << endl;

  for (int i = 0; i < fn.size(); i++) {
    cout << "inizio ciclo" << endl;

    auto file = fn.at(i);

    cv::Mat img = cv::imread(file);

    RobotProject rp(argc, argv);

    if (!rp.preprocessMap(img)) {
      std::cerr << "(Critical) Failed to preprocess map" << std::endl;
    }
    if (DEBUG) cvWaitKey(0);

    std::vector<double> state;
    cout << "localize" << endl;

    if (!rp.localize(img, state)) {
      std::cerr << "(Warning) Failed localization" << std::endl;
      continue;
    }
    if (DEBUG) cvWaitKey(0);

    Path path;
    if (!rp.planPath(img, path)) {
      std::cerr << "(Critical) Failed to plan path" << std::endl;
      return false;
    }
    if (DEBUG) cvWaitKey(0);

    cout << "mmm? " << endl;
    cvWaitKey(0);
  }

  // while (true) {
  //   std::vector<double> state;
  //   cout << "localize" << endl;

  //   if (!rp.localize(img, state)) {
  //     std::cerr << "(Warning) Failed localization" << std::endl;
  //     continue;
  //   }
  // }
}
