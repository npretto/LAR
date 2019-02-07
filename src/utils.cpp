#include "./utils.h"
#include "dubins.h"

using namespace std;
using namespace cv;

namespace u {

void dilate(const cv::Mat &image, int size) {
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size((size * 2) + 1, (size * 2) + 1));
  cv::dilate(image, image, kernel);
}

void erode(const cv::Mat &image, int size) {
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size((size * 2) + 1, (size * 2) + 1));
  cv::erode(image, image, kernel);
}

void blur(const cv::Mat &image, int size, int value) {
  cv::GaussianBlur(image, image, cv::Size(size, size), value, value);
}

void dilateErode(const cv::Mat &image, int size) {
  dilate(image, size);
  erode(image, size);
}

}  // namespace u
int printConfiguration(double q[3], double total, void *user_data) {
  vector<Point3f> &path = *(static_cast<vector<Point3f> *>(user_data));
  const double x = q[0];
  const double y = q[1];
  const double theta = q[2];

  Point2f a(x, y);

  Point2f b(x + cos(theta) * 5, y + sin(theta) * 5);

  path.push_back(Point3f(x, y, theta));

  // printf("%f, %f, %f, %f\n", q[0], q[1], q[2], total);
  return 0;
}

vector<Point3f> getDubinPath(const Point3f &a, const Point3f &b) {
  vector<Point3f> path;
  double q0[] = {a.x, a.y, a.z};
  double q1[] = {b.x, b.y, b.z};
  ;
  DubinsPath p;
  dubins_shortest_path(&p, q0, q1, TURNING_RADIUS);
  // cout << "PATH ,,,,,,,,,,,,,,,,,,,," << endl;
  dubins_path_sample_many(&p, 10, printConfiguration, &path);
  return path;
}

float point2angle(Point p) { return atan2(p.y, p.x); }

void filter(const cv::Mat &input, cv::Mat &output, string name) {
  FileStorage fs;
  fs.open(std::string("./filters/") + name + ".yml", FileStorage::READ);

  // fs["iterationNr"] >> itNr;
  auto node = fs["filter"];
  if (!node.isNone()) {
    vector<float> a, b;
    node["low"] >> a;
    node["high"] >> b;

    cout << "> " << a[0] << " " << a[1] << " " << a[2] << endl;
    cout << "> " << b[0] << " " << b[1] << " " << b[2] << endl;

    cv::Scalar low(a[0], a[1], a[2]);
    cv::Scalar high(b[0], b[1], b[2]);

    cv::inRange(input, low, high, output);
  } else {
    cout << "   FILTER  DOESN'T FIND THE FILE " << endl;
  }
}
