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

  std::vector<double> state;
  if (!arena.findRobot(img, state)) {
    cout << "ROBOT NOT FOUND, ABORT" << endl;
    return false;
  }

  int dist = 40;
  float theta = state.at(2);

  vector<Point3f> dubins = pf.testClick(state.at(0) + dist * sin(theta),
                                        state.at(1) + dist * cos(theta), theta);

  pf.drawMapOn(display);
  cv::imshow("Arena with path", display);

  std::vector<Pose> points;

  float distance = 0;

  for (auto p : dubins) {
    distance += 10;
    Pose pose(pixelToCm(distance) / 100, static_cast<double>(p.x),
              static_cast<double>(p.y), static_cast<double>(p.z),
              static_cast<double>(0.0));  // positivo a sinistra
    points.push_back(pose);
  }

  path.setPoints(points);

  return true;
}

// Method invoked periodically to determine the position of the robot within
// the map. The output state is a vector of three elements, x, y and theta.
bool RobotProject::localize(cv::Mat const& img, std::vector<double>& state) {
  return arena.findRobot(img, state);
}
// };
