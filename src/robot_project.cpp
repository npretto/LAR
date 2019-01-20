#pragma once
#include "./robot_project.h"

using namespace std;

// Define your own class RobotProject, that implements and exposes the following
// // methods. NB: The input images are already undistorted.
// class RobotProject {
//  private:
//   Arena arena;
//   PathFinder pf;

RobotProject::RobotProject(int argc, char* argv[]) {
  cout << "CONSTRUCTTOR OF ROBOT PROJECT " << endl;
}

// Method invoked to preprocess the map (extrinsic calibration +
// reconstruction of layout)
bool RobotProject::preprocessMap(cv::Mat const& img) {
  cout << "about to parse arena " << endl;

  if (arena.parseImage(img) == false) return false;

  cout << "arena parsed from image" << endl;

  cv::imshow("Arena", arena.topView);

  return true;
}

// Method invoked when a new path must be planned (detect initial robot
// position from img)
bool RobotProject::planPath(cv::Mat const& img, Path& path) {
  cv::Mat a(arena.topView.rows, arena.topView.cols, CV_8UC3,
            Scalar(100, 100, 100));

  display = a;

  pf.fromArena(arena);

  pf.drawMapOn(display);
  cv::imshow("Arena pf", display);

  pf.testClick(60, 60);

  pf.drawMapOn(display);
  cv::imshow("Arena with path", display);

  return true;
}

// Method invoked periodically to determine the position of the robot within
// the map. The output state is a vector of three elements, x, y and theta.
bool RobotProject::localize(cv::Mat const& img, std::vector<double>& state) {
  return false;
}
// };
