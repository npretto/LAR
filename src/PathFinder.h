
#pragma once
#include <iostream>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "./Arena.h"
#include "./DigitOCR.h"
#include "./main.h"
#include "./utils.h"

using namespace std;
using namespace cv;

struct CircumscribedObstacle {
  Point center;
  float innerRadius;
  float radius;
};

struct Edge;

struct GraphNode {
  Point center;
  bool ok;
  vector<Edge *> links;
  float cost;
  bool operator<(const GraphNode &rhs) const { return cost < rhs.cost; }
};

struct Compare {
  bool operator()(const GraphNode *a, const GraphNode *b) const {
    return a->cost > b->cost;
  }
};

float heuristic(GraphNode *start, GraphNode *end);

struct Edge {
  GraphNode *a;
  GraphNode *b;
  float cost;
};

class PathFinder {
 private:
  CircumscribedObstacle poligonToCircle(vector<cv::Point> points);

  // check che sia all'interno dell'arena e che sia fuori dagli ostacoli
  bool isPointValid(float x, float y, float buffer = 0);
  bool pathDoesNotCollide(vector<Point3f> points);

  bool pathVisitsAllPOIs(vector<Point3f> points);

  void createNodes(float distance);
  bool pointInsideObstacles(const Point &a);
  void createSmartNodes();

  void createEdges(float distance);

 public:
  Arena arena;
  vector<CircumscribedObstacle> obstacles;
  // vector<vector<GraphNode>> nodesMap;
  vector<GraphNode> nodes;
  vector<Edge *> edges;
  vector<vector<Point3f>> vectors;
  vector<Point3f> dubinsPath;

  void fromArena(Arena &a);

  void drawMapOn(const cv::Mat &image);

  // based on
  // https://www.redblobgames.com/pathfinding/a-star/implementation.html
  vector<GraphNode *> getAStarPath(GraphNode *start, GraphNode *goal);

  void drawPath(Mat image);

  GraphNode *getClosestNode(Point p);

  vector<Point3f> simplify(vector<Point3f> vectors);

  GraphNode *closestNodeToPOI(POI const &poi);

  vector<Point3f> nodesToVectorPath(vector<GraphNode *> nodes);
  vector<Point3f> testClick(int x, int y, float direction);
  vector<Point3f> vectorsToDubins(vector<Point3f> const &nodes);

  // thanks to http://paulbourke.net/geometry/circlesphere/
  bool lineIntersectsWithObstacles(const Point &a, const Point &b);
};
