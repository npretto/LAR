
#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include "../main.h"
#include "./Arena.cpp"
#include "./DigitOCR.cpp"
#include "./utils.cpp"

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

float heuristic(GraphNode *start, GraphNode *end) {
  // return 1;
  return cv::norm(start->center - end->center);
}

struct Edge {
  GraphNode *a;
  GraphNode *b;
  float cost;
};

class PathFinder {
 private:
  CircumscribedObstacle poligonToCircle(vector<cv::Point> points) {
    cv::Point2f center(0.0f, 0.0f);
    for (auto p : points) {
      center.x += p.x;
      center.y += p.y;
    }

    center.x = center.x / points.size();
    center.y = center.y / points.size();

    float maxRadius = 0;
    for (Point2f p : points) {
      float radius = cv::norm(center - p);
      if (radius > maxRadius) maxRadius = radius;
    }

    return CircumscribedObstacle{center, maxRadius, maxRadius + robotRadius};
  }

  // check che sia all'interno dell'arena e che sia fuori dagli ostacoli
  bool isPointValid(float x, float y, float buffer = 0) {
    if (!arena.isPointInside(x, y, buffer)) return false;

    for (CircumscribedObstacle o : obstacles) {
      if (norm(Point(x, y) - o.center) < o.radius) return false;
    }
    return true;
  }
  bool isPathValid(vector<Point3f> points) {
    for (const auto &p : points) {
      if (!isPointValid(p.x, p.y, 0)) return false;
    }
    return true;
  }
  void createNodes(float distance) {
    int cols = arena.getWidth() / distance;
    int rows = arena.getHeight() / distance;
    for (int row = 0; row < rows; row++) {
      vector<GraphNode> rowVector;
      for (int col = 0; col < cols; col++) {
        bool ok = true;
        Point p(col * distance + distance / 2, row * distance + distance / 2);
        // force field degli ostacoli
        Point forceSum(0, 0);
        for (CircumscribedObstacle o : obstacles) {
          Point distancePoint = p - o.center;
          float distance = cv::norm(distancePoint);
          if (distance > o.radius) {  // outside the obstacle
                                      // forceSum += 1 / (distance - o.radius) *
                                      // distancePoint;
          } else {                    // inside the obstacle
            if (distance < o.innerRadius) {
              ok = false;
              continue;
            } else {
              forceSum = (o.radius - distance + safetyDistance) *
                         distancePoint / cv::norm(distancePoint);
            }
          }
        }

        Point position = ok ? p + forceSum : p;
        if (arena.isPointInside(position.x, position.y, 15)) {
          GraphNode node{position, ok};
          // rowVector.push_back(node);
          if (ok) nodes.push_back(node);
        }
      }
      // nodesMap.push_back(rowVector);
    }
  }
  bool pointInsideObstacles(const Point &a) {
    for (CircumscribedObstacle c : obstacles) {
      const Point p = c.center;

      if (cv::norm(a - p) < (c.radius)) return true;
    }
    return false;
  }
  void createSmartNodes() {
    const float distance = 20;

    for (CircumscribedObstacle o : obstacles) {
      const int steps = 2 * 3.14 * o.radius / distance;
      const float step = 2 * 3.14 / steps;

      for (int i = 0; i < steps; i++) {
        const float angle = step * i;
        Point2f dir(cos(angle) * (o.radius + safetyDistance),
                    sin(angle) * (o.radius + safetyDistance));
        cout << angle << "(" << cos(angle) << ", " << dir.y << ")" << endl;
        const Point pos = o.center + Point((int)dir.x, (int)dir.y);
        if (isPointValid(pos.x, pos.y, 15)) {
          GraphNode node{pos, true};
          nodes.push_back(node);
        }
      }
    }

    // for (int i = 0; i < obstacles.size(); i++) {
    //   for (int j = i; j < obstacles.size(); j++) {
    //     CircumscribedObstacle *a = &obstacles[i];
    //     CircumscribedObstacle *b = &obstacles[j];

    //     Point dist = a->center - b->center;

    //     Point middle = (a->center + b->center) / 2;
    //     if (!pointInsideObstacles(middle)) {
    //       nodes.push_back(GraphNode{middle});
    //     }
    //   }
    // }
  }

  void createEdges(float distance) {
    for (int i = 0; i < nodes.size(); i++) {
      for (int j = i; j < nodes.size(); j++) {
        GraphNode *a = &nodes[i];
        GraphNode *b = &nodes[j];

        float dist = cv::norm(a->center - b->center);

        if (dist < distance &&
            !lineIntersectsWithObstacles(a->center, b->center)) {
          Edge *e = new Edge{a, b, heuristic(a, b)};
          a->links.push_back(e);
          b->links.push_back(e);

          edges.push_back(e);
        }
      }
    }
  }

 public:
  Arena arena;
  vector<CircumscribedObstacle> obstacles;
  // vector<vector<GraphNode>> nodesMap;
  vector<GraphNode> nodes;
  vector<Edge *> edges;
  vector<Point3f> vectors;
  vector<Point3f> dubinsPath;

  void fromArena(Arena a) {
    arena = a;

    for (auto polygon : arena.obstacles) {
      obstacles.push_back(poligonToCircle(polygon));
    }

    const float dist = NODES_DISTANCE;
    // node for each POI
    for (auto poi : arena.POIs) {
      nodes.push_back(GraphNode{Point(poi.position.x, poi.position.y), true});
    }
    if (SMART_NODES) {
      createSmartNodes();
      createNodes(cmToPixels(10));
      createEdges(cmToPixels(20));

    } else {
      createNodes(dist);
      createEdges(dist * 3.17);
    }
  }

  void drawMapOn(const cv::Mat &image) {
    // circles that contains obstacles
    for (CircumscribedObstacle obstacle : obstacles) {
      circle(image, obstacle.center, obstacle.innerRadius, Scalar(20, 20, 100),
             1, LINE_AA);
      circle(image, obstacle.center, obstacle.radius, Scalar(20, 20, 255), 2,
             LINE_AA);
    }

    // edges
    if (DRAW_EDGES) {
      for (Edge *e : edges) {
        line(image, e->a->center, e->b->center, Scalar(120, 120, 120), 1);
      }
    }

    // nodes
    for (GraphNode node : nodes) {
      // if (node.ok)
      circle(image, node.center, 2, Scalar(255, 255, 255), 1, LINE_AA);
      // else circle(image, node.center, 2, Scalar(0, 0, 255), 1, LINE_AA);
    }
  }

  // based on
  // https://www.redblobgames.com/pathfinding/a-star/implementation.html
  vector<GraphNode *> getAStarPath(GraphNode *start, GraphNode *goal) {
    vector<GraphNode *> path;

    std::unordered_map<GraphNode *, GraphNode *> came_from;
    std::unordered_map<GraphNode *, float> cost_so_far;

    priority_queue<GraphNode *, vector<GraphNode *>, Compare> frontier;

    start->cost = 0;
    frontier.push(start);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
      GraphNode *current = frontier.top();
      frontier.pop();

      if (current == goal) {
        break;
      }
      for (Edge *link : current->links) {
        if (DRAW_VISITED_EDGES) {
          line(display, link->a->center, link->b->center, Scalar(120, 120, 120),
               2);
        }

        GraphNode *next = link->a == current ? link->b : link->a;

        float new_cost = cost_so_far[current] + link->cost;
        if (cost_so_far.find(next) == cost_so_far.end() ||
            new_cost < cost_so_far[next]) {
          cost_so_far[next] = new_cost;

          float priority = new_cost + heuristic(next, goal);
          next->cost = priority;
          frontier.push(next);
          came_from[next] = current;
        }
      }
    }
    GraphNode *n = goal;
    path.push_back(goal);
    while (n != start) {
      GraphNode *prev = came_from[n];
      path.push_back(prev);
      n = prev;
    }
    reverse(begin(path), end(path));
    return path;
  }

  void drawPath(Mat image, vector<GraphNode *> path) {
    // for (int i = 0; i < path.size() - 1; i++) {
    //   auto a = path[i]->center;
    //   auto b = path[i + 1]->center;

    //   line(image, path[i]->center, path[i + 1]->center,
    //        Scalar(rand() * 120, rand() * 120, rand() * 255), 3);
    // }

    //  VECTORS
    for (Point3f vector : vectors) {
      const Scalar randColor(rand() * 255, rand() * 255, rand() * 255);
      const auto &a = vector;
      Point2f b(a.x + cos(a.z) * 80, a.y + sin(a.z) * 80);
      line(display, Point(a.x, a.y), b, randColor, 3);
      circle(display, Point(vector.x, vector.y), 4, Scalar(255, 255, 255), 5,
             LINE_AA);
      // cv::imshow("Arena parsed", display);
      // cvWaitKey(300);
    }

    // DUBINS PATH
    for (int i = 0; i < dubinsPath.size(); i++) {
      auto a = dubinsPath[i];
      Point2f b(a.x + cos(a.z) * 5, a.y + sin(a.z) * 5);

      circle(display, Point(a.x, a.y), 1, Scalar(255, 255, 255), 3, LINE_AA);
      line(display, Point(a.x, a.y), b, Scalar(10, 10, 10), 4);

      cv::imshow("Arena parsed", display);
      cout << "WAIT" << endl;
      cvWaitKey(30);
    }
  }

  GraphNode *getClosestNode(Point p) {
    GraphNode *closest;
    float minDistance = 10000;
    for (GraphNode &n : nodes) {
      float dist = cv::norm(n.center - p);
      if (dist < minDistance) {
        minDistance = dist;
        closest = &n;
      }
    }
    cout << closest->center.x << ", " << closest->center.y << endl;
    return closest;
  }

  vector<Point3f> vectorsToDubins(vector<Point3f> const &nodes) {
    vector<Point3f> dubinsPath;

    int start = 0;
    do {
      int end = start + 1;
      // vector<Point3f> path = getDubinPath(nodes[start], nodes[end]);
      // while (!isPathValid(path) && end < nodes.size()) {
      //   path = getDubinPath(nodes[start], nodes[++end]);
      // }

      // trovo un path valido
      vector<Point3f> validPath = getDubinPath(nodes[start], nodes[end]);
      while (!isPathValid(validPath) && end < nodes.size()) {
        validPath = getDubinPath(nodes[start], nodes[++end]);
      }

      // vado avanti finchè è valido
      int validEnd = end;
      vector<Point3f> attemptedPath;
      bool isValid;
      do {
        attemptedPath = getDubinPath(nodes[start], nodes[++end]);

        isValid = isPathValid(attemptedPath);
        if (isValid) {
          validPath = attemptedPath;
          validEnd = end;
        }
      } while (isValid && end < nodes.size() && (end - start) < 3);

      for (auto a : validPath) dubinsPath.push_back(a);
      start = validEnd;
    } while (start < nodes.size() - 1);

    return dubinsPath;
  }

  GraphNode *closestNodeToPOI(POI const &poi) {
    Point point = Point(poi.position.x, poi.position.y);
    return getClosestNode(point);
  }

  vector<Point3f> nodesToVectorPath(vector<GraphNode *> nodes) {
    vector<Point3f> vectors;

    float theta;

    for (int i = 0; i < nodes.size() - 1; i++) {
      auto &a = nodes[i]->center;
      auto &b = nodes[i + 1]->center;

      theta = point2angle(b - a);

      Point3f p(a.x, a.y, theta);
      vectors.push_back(p);
    }
    const auto &last = vectors[vectors.size() - 1];
    vectors.push_back(Point3f(last.x, last.y, theta));
    return vectors;
  }

  vector<GraphNode *> testClick(int x, int y) {
    cout << x << "  " << y << endl;
    // const Point a(300, 350);
    GraphNode *start = getClosestNode(Point(x, y));

    vector<GraphNode *> totalNodesPath =
        getAStarPath(start, closestNodeToPOI(arena.POIs.at(0)));
    for (int i = 0; i < arena.POIs.size() - 1; i++) {
      GraphNode *a = closestNodeToPOI(arena.POIs.at(i));
      GraphNode *b = closestNodeToPOI(arena.POIs.at(i + 1));
      cout << "getting path from " << arena.POIs.at(i).c << " to  "
           << arena.POIs.at(i + 1).c << endl;
      const auto partialPath = getAStarPath(a, b);

      totalNodesPath.insert(totalNodesPath.end(), partialPath.begin(),
                            partialPath.end());
    }

    //"vectors" = points with directions
    vectors = nodesToVectorPath(totalNodesPath);

    dubinsPath = vectorsToDubins(vectors);

    return totalNodesPath;
  }

  // thanks to http://paulbourke.net/geometry/circlesphere/
  bool lineIntersectsWithObstacles(const Point &a, const Point &b) {
    for (CircumscribedObstacle c : obstacles) {
      const Point p = c.center;

      if (cv::norm(a - p) < (c.radius)) return true;
      if (cv::norm(b - p) < (c.radius)) return true;

      const float x1 = a.x;
      const float x2 = b.x;
      const float x3 = p.x;
      const float y1 = a.y;
      const float y2 = b.y;
      const float y3 = p.y;

      float aa = pow(x2 - x1, 2) + pow(y2 - y1, 2);
      float bb = 2 * ((x2 - x1) * (x1 - x3) + (y2 - y1) * (y1 - y3));
      float cc = pow(x3, 2) + pow(y3, 2) + pow(x1, 2) + pow(y1, 2) -
                 2 * (x3 * x1 + y3 * y1) - pow(c.radius, 2);

      float bb4ac = bb * bb - 4 * aa * cc;
      if (bb4ac < 0) {
        // no contacts between line and circle
        continue;
      }

      const float u = ((x3 - x1) * (x2 - x1) + (y3 - y1) * (y2 - y1)) /
                      ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
      if (u >= 0 && u <= 1) {
        // closest point of the line to the center is between a and b
        // this is still false if i don't "go over" the center
        // that's why i also check (at the start) if a or b are in the circle
        return true;
      }
    }
    return false;
  }
};
