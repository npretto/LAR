
#include "./Arena.h"

using namespace std;
using namespace cv;

/// get random color util
cv::Scalar Arena::getColor()  // get "random" color to draw
{
  static int i = 0;
  cv::Scalar colors[] = {cv::Scalar(200, 10, 10), cv::Scalar(0, 200, 0),
                         cv::Scalar(0, 0, 200)};

  return colors[i++ % 3];
}

float Arena::getWidth() { return topView.cols; }
float Arena::getHeight() { return topView.rows; }

/// check if poing is inside the arena, buffer is the distance from the edges
bool Arena::isPointInside(float x, float y, float buffer) {
  if (x < buffer || y < buffer) return false;
  if (x > (getWidth() - buffer)) return false;
  if (y > (getHeight() - buffer)) return false;

  return true;
}

//parse the input image, display = should show images
bool Arena::parseImage(cv::Mat input, bool display) {
  obstacles.clear();
  goal.clear();
  POIs.clear();
  goalCenter.x = 0;
  goalCenter.y = 0;

  getTopView(input, false);
  // if (!getTopViewAt16cm(input, true)) return false;

  cout << "find obstacles" << endl;
  findObstacles(false);

  cout << "findGoal" << endl;
  findGoal(false);

  cout << "findPOIs" << endl;
  findPOIs(false);

  return true;
}

/// util function to sort pois alphabetically by the char
bool Arena::comparePOI(const POI &p1, const POI &p2) { return p1.c < p2.c; }


///looks for Points of interests, display flag for showing images for debug purposes
void Arena::findPOIs(bool display) {
  cv::Mat green_mask;

  const int color = 150 / 2;
  // cv::inRange(topView_hsv, cv::Scalar(color - 15, 60, 10),
  //             cv::Scalar(color + 15, 255, 255), green_mask);
  filter(topView_hsv, green_mask, "pois");

  if (display)
    if (DEBUG) cv::imshow("green_mask", green_mask);

  u::erode(green_mask, 5);
  u::dilate(green_mask, 1);

  std::vector<Vec3f> circles;

  cv::HoughCircles(green_mask, circles, HOUGH_GRADIENT, 1, 30, 100, 10, 30, 60);

  ocr.init();
  for (size_t i = 0; i < circles.size(); i++) {
    Vec3i c = circles[i];

    const int margin = 2;  // margin to add to the area

    cv::Rect r(c[0] - c[2] - margin, c[1] - c[2] - margin, (c[2] + margin) * 2,
               (c[2] + margin) * 2);
    cv::Mat digitArea = topView(r);

    char digit = ocr.parse(digitArea);

    POIs.push_back(POI{Point3f(c[0], c[1], c[2]), digit});

    if (STOP_AT_EVERY_OCR) cv::waitKey();
  }

  sort(POIs.begin(), POIs.end(), Arena::comparePOI);

  // for (auto p : POIs) {
  //   cout << p.position << " " << p.c << endl;
  // }

  if (display) {
    if (DEBUG) cv::imshow("green_mask_eroded", green_mask);
  }
}

/// looks for obstacles in the arena, display flag for showing images for debug purposes
void Arena::findObstacles(bool display) {
  obstacles.clear();

  cv::Mat red_mask;
  cv::Mat red_mask2;

  cv::cvtColor(topView, topView_hsv, cv::COLOR_BGR2HSV);

  const int range = 20;
  const int S = 0;
  const int V = 0;
  cv::inRange(topView_hsv, cv::Scalar(0, S, V), cv::Scalar(range, 255, 255),
              red_mask);
  cv::inRange(topView_hsv, cv::Scalar(180 - range, S, V),
              cv::Scalar(180, 255, 255), red_mask2);
  red_mask = red_mask | red_mask2;

  if (display)
    if (DEBUG) cv::imshow("red_mask", red_mask);
  u::erode(red_mask, 2);
  // u::dilateErode(red_mask);
  // u::erode(red_mask, 2);

  // u::dilateErode(red_mask);

  std::vector<std::vector<cv::Point>> contours, approximation;

  cv::findContours(red_mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  for (int i = 0; i < contours.size(); ++i) {
    std::vector<cv::Point> approx_curve;

    approxPolyDP(contours[i], approx_curve, 5, true);
    approximation = {approx_curve};

    obstacles.push_back(approx_curve);
  }

  if (display) {
    if (DEBUG) cv::imshow("red_mask_eroded", red_mask);
  }
}
/// looks for the goal in the arena , display flag for showing images for debug purposes
void Arena::findGoal(bool display) {
  cv::Mat blue_mask;

  // cv::inRange(topView_hsv, cv::Scalar(220 / 2 - 55, 100, 50),
  //             cv::Scalar(220 / 2 + 25, 255, 255), blue_mask);
  filter(topView_hsv, blue_mask, "goal");

  if (display)
    if (DEBUG) cv::imshow("blue_mask", blue_mask);

  u::dilateErode(blue_mask);

  std::vector<std::vector<cv::Point>> contours, approximation;

  cv::findContours(blue_mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  // drawContours(topViewAnnotated, contours, 0, cv::Scalar(255, 20, 20), 3,
  // cv::LINE_AA);

  std::vector<cv::Point> approx_curve;
  for (int i = 0; i < contours.size(); ++i) {
    approxPolyDP(contours[i], approx_curve, 5, true);
    approximation = {approx_curve};

    if (display)
      drawContours(topViewAnnotated, approximation, -1, getColor(), 2,
                   cv::LINE_AA);

    double area = contourArea(contours[i]);

    if (area > 40) {
      goal = approx_curve;
    }
  }

  cout << " aa " << endl;
  for (cv::Point p : approx_curve) {
    goalCenter += p;
  }
  cout << " bb " << endl;

  goalCenter.x = goalCenter.x / approx_curve.size();
  goalCenter.y = goalCenter.y / approx_curve.size();
  cout << " cc " << endl;

  std::vector<std::vector<cv::Point>> a = {approx_curve};

  // cvWaitKey();
  drawContours(topViewAnnotated, a, -1, cv::Scalar(20, 20, 255), 2,
               cv::LINE_AA);

  if (display)
    if (DEBUG) cv::imshow("blue_mask_eroded", blue_mask);
}

///looks for the 4 white circles to parse the arena at the 16cm level, cropped/warped image is put in output
bool Arena::getTopViewAt16cm(cv::Mat input, cv::Mat &output, bool debugView) {
  if (transform16cm.cols == 0) {
    cv::Mat topView_hsv, white_mask;
    cv::cvtColor(input, topView_hsv, cv::COLOR_BGR2HSV);

    // cv::inRange(topView_hsv, cv::Scalar(70, 0, 150), cv::Scalar(240, 50,
    // 255),
    //             white_mask);

    // cv::inRange(topView_hsv, cv::Scalar(70, 0, 150), cv::Scalar(240, 70,
    // 255),
    //             white_mask);

    filter(topView_hsv, white_mask, "white-circles");

    if (false)
      if (DEBUG) cv::imshow("gray_mask", white_mask);

    // u::erode(white_mask, 8);
    // u::dilate(white_mask, 3);
    // u::erode(white_mask, 3);
    // u::dilate(white_mask, 3);
    u::blur(white_mask, 5, 5);

    std::vector<Vec3f> circles;

    cv::HoughCircles(white_mask, circles, HOUGH_GRADIENT,
                     1,    // a
                     30,   // min distance
                     30,   // p1
                     20,   // p2 : lower => more circles
                     14,   // min radius
                     25);  // max radius

    cout << "trovati " << circles.size() << " cerchi" << endl;

    cv::Mat circles_pic(white_mask.rows, white_mask.cols, CV_8UC3,
                        Scalar(100, 100, 100));

    for (size_t i = 0; i < circles.size(); i++) {
      Vec3i c = circles[i];

      circle(circles_pic, Point(c[0], c[1]), c[2], Scalar(255, 100, 100), 5,
             LINE_AA);
    }

    if (false)
      if (DEBUG) cv::imshow("circles", circles_pic);

    if (circles.size() != 4) return false;

    // "flatten" the map to a rectangular image
    // std::vector<cv::Point2f> desidered = {
    //     cv::Point2f(width, 0), cv::Point2f(width, height),
    //     cv::Point2f(0, height), cv::Point2f(0, 0)

    // };

    std::vector<cv::Point2f> desidered = {
        cv::Point2f(width, 0), cv::Point2f(width, height),
        cv::Point2f(0, height), cv::Point2f(0, 0)

    };

    vector<Point2f> detected;
    Point2f center(0, 0);
    for (auto c : circles) {
      center += Point2f(c[0], c[1]);
      cout << c[0] << ", " << c[1] << endl;
      detected.push_back(Point2f(c[0], c[1]));
    }
    center = center / 4;

    for (auto &p : detected) {
      p -= center;
    }

    sort(detected.begin(), detected.end(), sort_atan());

    for (auto &p : detected) {
      p += center;
    }

    // reverse(detected.begin(), detected.end());

    transform16cm = getPerspectiveTransform(detected, desidered);
  }

  cv::warpPerspective(input, output, transform16cm, cv::Size(width, height));

  // if(DEBUG) imshow("topViewAt16cm", output);

  return true;
}

/// looks for the topview cropped to the arena (inside sides of black tape) and stores it in topView
void Arena::getTopView(cv::Mat input, bool debugView) {
  cv::Mat topView_hsv, black_mask;
  cv::cvtColor(input, topView_hsv, cv::COLOR_BGR2HSV);

  cv::inRange(topView_hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 80),
              black_mask);

  if (debugView)
    if (DEBUG) cv::imshow("BLACK_filter", black_mask);

  // Find contours
  std::vector<std::vector<cv::Point>> contours, approximation;
  std::vector<cv::Point> arena, arena_approx, largest, largest_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;

  // Apply some filtering
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size((4 * 2) + 1, (4 * 2) + 1));
  cv::dilate(black_mask, black_mask, kernel);
  cv::erode(black_mask, black_mask, kernel);

  // Process black mask
  cv::findContours(
      black_mask, contours, cv::RETR_LIST,
      cv::CHAIN_APPROX_SIMPLE);  // find external contours of each blob
  contours_img = input.clone();
  // drawContours(contours_img, contours, -1, cv::Scalar(40, 190, 40), 1,
  // cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;

  double maxArea = 0;
  double arenaArea = 0;

  int epsilon = 30;
  int maxEdges = 8;

  // i take the second biggest polygon, (the biggest is the outer contour)
  for (int i = 0; i < contours.size(); ++i) {
    approxPolyDP(contours[i], approx_curve, epsilon, true);
    approximation = {approx_curve};
    cout << ".";

    if (debugView) {
      drawContours(contours_img, approximation, -1, getColor(), 2, cv::LINE_AA);
    }
    if (debugView)
      if (DEBUG) cv::imshow("contours_img", contours_img);
    // cvWaitKey();

    double area = contourArea(contours[i]);

    if (area > maxArea /*&& approximation[0].size() < maxEdges*/) {
      arenaArea = area;
      arena_approx = largest_approx;
      arena = largest;

      maxArea = area;
      largest_approx = approx_curve;
      largest = contours[i];
    }
  }

  // arena = largest;
  // arena_approx = largest_approx;

  // cvWaitKey();
  std::cout << "largest countours " << largest_approx.size() << std::endl;

  std::vector<std::vector<cv::Point>> const a = {arena_approx};

  // drawContours(contours_img, a, -1, getColor(), 20, cv::LINE_AA);
  // if(DEBUG) cv::imshow("contours_img", contours_img);

  cout << "arena_approx.size: " << arena_approx.size() << endl;

  // in case it has more than 4 edges for some reason, i reduce it to 4
  // edges, it should happen only with the outer border but i'll keep it
  // here for safety
  int eps = epsilon;
  while (arena_approx.size() > 4) {
    approxPolyDP(arena_approx, arena_approx, eps, true);
    cout << "epsilon to : -> " << eps << endl;
    cout << "reduced to : -> " << arena_approx.size() << endl;
    eps *= 1.10;
  }

  std::vector<std::vector<cv::Point>> aa = {arena_approx};
  std::vector<std::vector<cv::Point>> bb = {arena};

  // "flatten" the map to a rectangular image
  // std::vector<cv::Point2f> desidered = {
  //     cv::Point2f(0, 0),
  //     cv::Point2f(width, 0),
  //     cv::Point2f(width, height),
  //     cv::Point2f(0, height),
  // };

  // // detect if the image is "rotated"
  // if (cv::norm(arena_approx.at(0) - arena_approx.at(1)) >
  //     cv::norm(arena_approx.at(1) - arena_approx.at(2))) {
  //   cout << "IMAGE IS ROTATED " << endl;
  //   desidered.push_back(desidered.at(0));
  //   desidered.erase(desidered.begin());
  // }

  std::vector<cv::Point2f> desidered = {
      cv::Point2f(width, 0), cv::Point2f(width, height), cv::Point2f(0, height),
      cv::Point2f(0, 0)

  };

  vector<Point2f> detected;
  Point2f center(0, 0);
  for (auto c : arena_approx) {
    center += Point2f(c.x, c.y);
    cout << c.x << ", " << c.y << endl;
    detected.push_back(Point2f(c.x, c.y));
  }
  center = center / 4;

  for (auto &p : detected) {
    p -= center;
  }

  sort(detected.begin(), detected.end(), sort_atan());

  for (auto &p : detected) {
    p += center;
  }

  // reverse(detected.begin(), detected.end());

  cv::Mat transform = getPerspectiveTransform(detected, desidered);

  cv::warpPerspective(input, topView, transform, cv::Size(width, height));

  topViewAnnotated = topView.clone();

  cv::cvtColor(topView, topView_hsv, cv::COLOR_BGR2HSV);

  return;
}

/// draw the current map representation on the image param
void Arena::drawMapOn(cv::Mat &image) {
  // DRAW POIs
  for (size_t i = 0; i < POIs.size(); i++) {
    Point3f c = POIs[i].position;
    // cout << c[2] << endl;
    const int margin = 2;  // margin to add to the area
    circle(image, Point(c.x, c.y), c.z, Scalar(20, 255, 20), 1, LINE_AA);
    circle(image, Point(c.x, c.y), 2, Scalar(0, 255, 0), 3, LINE_AA);

    char number = POIs[i].c;
    putText(image, string(1, number), cvPoint(c.x, c.y),
            FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(255, 255, 255), 4, CV_AA);
  }

  // DRAW OBSTACLES
  cout << ">>>>" << endl;
  cout << obstacles.size() << endl;
  drawContours(image, obstacles, -1, cv::Scalar(20, 20, 180), 3, cv::LINE_AA);

  // DRAW EXIT
  std::vector<std::vector<cv::Point>> a = {goal};

  drawContours(image, a, -1, cv::Scalar(255, 20, 20), 3, cv::LINE_AA);
}

/// looks for the robot in img and puts the position in state
bool Arena::findRobot(cv::Mat const &img, std::vector<double> &state) {
  const bool display = false;
  cout << " 10" << endl;

  cv::Mat topViewRobotAt16(img.rows, img.cols, CV_8UC3, Scalar(100, 100, 100));
  cout << " 20" << endl;

  getTopViewAt16cm(img, topViewRobotAt16, false);
  cout << " 30" << endl;

  if (DEBUG) cv::imshow("topViewRobotAt16", topViewRobotAt16);
  cv::Mat hsv, blue_mask;
  cout << " 40" << endl;

  cv::cvtColor(topViewRobotAt16, hsv, cv::COLOR_BGR2HSV);

  const int target = 184 / 2;

  // cv::inRange(hsv, cv::Scalar(target - 25, 90, 90),
  //             cv::Scalar(target + 35, 190, 190), blue_mask);

  filter(hsv, blue_mask, "robot");

  if (display)
    if (DEBUG) cv::imshow("blue_mask", blue_mask);

  // cvWaitKey(0);

  // u::erode(blue_mask, 1);
  u::blur(blue_mask, 3, 3);
  // u::dilate(blue_mask, 2);

  std::vector<std::vector<cv::Point>> contours, approximation;

  cout << " robot_projecte.cpp 01" << endl;

  cv::findContours(blue_mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
  cout << " robot_projecte.cpp 02" << endl;

  drawContours(topViewRobotAt16, contours, -1, cv::Scalar(255, 20, 20), 3,
               cv::LINE_AA);
  cout << " robot_projecte.cpp 03" << endl;

  if (display)
    if (DEBUG) cv::imshow("topViewRobotAt16", topViewRobotAt16);
  if (display)
    if (DEBUG) cv::imshow("blue_mask_eroded", blue_mask);

  // cvWaitKey(0);

  std::vector<cv::Point> robot;

  std::vector<cv::Point> approx_curve;
  for (int i = 0; i < contours.size(); ++i) {
    approxPolyDP(contours[i], approx_curve, 10, true);
    approximation = {approx_curve};
    cout << " robot_projecte.cpp 40+++" << endl;

    if (display)
      drawContours(topViewRobotAt16, approximation, -1, cv::Scalar(20, 20, 255),
                   1, cv::LINE_AA);
    if (display)
      if (DEBUG) cv::imshow("topViewRobotAt16", topViewRobotAt16);

    double area = contourArea(contours[i]);
    cout << "> " << approx_curve.size() << "  " << area << endl;
    if (area > 1000 && approx_curve.size() == 3) {
      robot = approx_curve;
      cout << "TROVATO ROBOT" << endl;
    }
  }
  cout << " robot_projecte.cpp 50" << endl;
  if (display)
    if (DEBUG) cv::imshow("topViewRobotAt16", topViewRobotAt16);
  // cvWaitKey(0);

  std::vector<std::vector<cv::Point>> a = {robot};
  if (robot.size() > 0)
    drawContours(topViewRobotAt16, a, -1, cv::Scalar(255, 255, 255), 2,
                 cv::LINE_AA);

  cout << " robot_projecte.cpp 55" << endl;

  cout << " robot_projecte.cpp 60 guarda mappa" << endl;
  // cvWaitKey(0);

  if (robot.size() == 3) {
    cout << "robot trovato" << endl;
    auto b = (robot.at(0) + robot.at(1) + robot.at(2)) / 3;

    Point front;

    auto l1 = norm(robot.at(0) - robot.at(1));
    auto l2 = norm(robot.at(1) - robot.at(2));
    auto l3 = norm(robot.at(2) - robot.at(0));

    if (l1 < l2 && l1 < l3) {
      front = (robot.at(0) + robot.at(1)) / 2;
    } else if (l2 < l1 && l2 < l3) {
      front = (robot.at(1) + robot.at(2)) / 2;
    } else if (l3 < l1 && l3 < l2) {
      front = (robot.at(2) + robot.at(0)) / 2;
    }

    auto direction = front - b;

    state.push_back(b.x);
    state.push_back(b.y);
    state.push_back(atan2(direction.y, direction.x));

    if (display)
      circle(topViewRobotAt16, b, 4, Scalar(255, 255, 255), 5, LINE_AA);

    if (display)
      circle(topViewRobotAt16, front, 3, Scalar(120, 255, 120), 3, LINE_AA);

    if (display)
      if (DEBUG) cv::imshow("topViewRobotAt16", topViewRobotAt16);

    // cvWaitKey(0);
    // cvWaitKey(0);
    // cvWaitKey(0);

    return true;
  } else {
    return false;
  }
}
