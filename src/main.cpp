#ifndef MAIN
#define MAIN

#include "./Arena.cpp"
#include "./CalibratedCamera.cpp"
#include "./PathFinder.cpp"

const String CALIBRATION_PATH = "./data/calibration";
const bool calibrate = false;
bool STOP_AT_EVERY_OCR = false;
bool DRAW_EDGES = true;
bool DRAW_VISITED_EDGES = true;

bool SMART_NODES = false;
float NODES_DISTANCE = cmToPixels(9);
float MAP_HEIGHT = 150;  // 150cm
float MAP_WIDTH = 100;   // 100cm
float TURNING_RADIUS = cmToPixels(15);

float pixelToCm(float pixels) { return pixels / Arena::width * MAP_WIDTH; }
float cmToPixels(float cm) { return cm / MAP_WIDTH * Arena::width; }

float safetyDistance = cmToPixels(3);
float robotRadius = 30;

CalibratedCamera camera;
Arena *arena;
PathFinder *pf;

cv::Mat display;

static void click(int x, int y) {
  cv::Mat display2(arena->topView.rows, arena->topView.cols, CV_8UC3,
                   Scalar(100, 100, 100));
  display = display2;

  arena->drawMapOn(display);

  // vector<GraphNode *> path =
  pf->testClick(x, y);
  pf->drawMapOn(display);
  pf->drawPath(display);

  cv::imshow("Arena parsed", display);

  cout << "." << endl;
}
static void onMouse(int event, int x, int y, int, void *) {
  // click(x, y);
}

int main_OLD(int argc, char **argv) {
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
    cout << "inizio ciclo" << endl;
    arena = new Arena();
    pf = new PathFinder();
    cout << "creati nuovi oggetti" << endl;

    auto file = fn.at(i);
    cout << "Undistorting FILE: " << file << endl;
    cv::Mat image = cv::imread(file);
    // camera.undistort(image, image);
    arena->parseImage(image);
    cout << "ARENA PARSED without crashing" << endl;
    cv::imshow("Arena", arena->topView);

    cv::Mat display = arena->topView;
    // cv::Mat display(arena.topView.rows, arena.topView.cols, CV_8UC3,
    //                 Scalar(100, 100, 100));
    cout << "ARENA . DRAW MAP ON " << endl;
    arena->drawMapOn(display);

    cv::imshow("Arena parsed", display);

    // cvWaitKey();

    pf->fromArena(*arena);
    pf->drawMapOn(display);

    // cv::imshow("Arena parsed", display);

    // setMouseCallback("Arena parsed", onMouse, 0);
    cout << "CLICK" << endl;
    // cvWaitKey();
    click(60, 60);

    cout << "SHOULD PRESS FOR NEXT MAP?" << endl;
    waitKey();
    // for (;;) {
    //   // waitKey(10);
    // }

    // delete arena;
    // cout << "DELETE FATTO per arena" << endl;
    // cout << pf << endl;
    // delete pf;
    // cout << "DELETE FATTO per pf" << endl;
  }

  waitKey(0);
  return 0;
}

#endif
