#ifndef PATH_H
#define PATH_H

#include <vector>
#include <cmath>

// A configuration of the robot along the path, represented by x, y, orientation and curvature
struct Pose 
{
  double s, x, y, theta, kappa;

  Pose(double s, double x, double y, double theta, double kappa):
    s(s), x(x), y(y), theta(theta), kappa(kappa)
  {}

  Pose(): 
    Pose(0, 0, 0, 0, 0)
  {}

  double distance(double _x, double _y)
  {
    return std::hypot(x-_x, y-_y);
  }
};

// A sequence of sampled robot configurations composing a (discretization of the) path
struct Path 
{
  std::vector<Pose> points;
  
  Path(std::vector<Pose> const & points):
    points(points)
  {}

  Path()
  {}
  
  bool empty() { return points.empty(); }
  size_t size() { return points.size(); }
  void setPoints(const std::vector<Pose>& points) { this->points = points; }
};

#endif 