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

  arena.parseImage(img);
  cout << "arena parsed from image" << endl;

  return true;
}

// Method invoked when a new path must be planned (detect initial robot
// position from img)
bool RobotProject::planPath(cv::Mat const& img, Path& path) {
  // aa
  return false;
}

// Method invoked periodically to determine the position of the robot within
// the map. The output state is a vector of three elements, x, y and theta.
bool RobotProject::localize(cv::Mat const& img, std::vector<double>& state) {
  return false;
}
// };
