
#include "./PathFinder.h"

using namespace std;
using namespace cv;

float heuristic(GraphNode *start, GraphNode *end) {
  // return 1;
  return cv::norm(start->center - end->center);
}

CircumscribedObstacle PathFinder::poligonToCircle(vector<cv::Point> points) {
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
bool PathFinder::isPointValid(float x, float y, float buffer) {
  if (norm(Point(x, y) - arena.goalCenter) < cmToPixels(15)) return true;

  if (!arena.isPointInside(x, y, buffer)) return false;

  for (CircumscribedObstacle o : obstacles) {
    if (norm(Point(x, y) - o.center) < o.radius) return false;
  }
  return true;
}

bool PathFinder::pathDoesNotCollide(vector<Point3f> points) {
  vector<bool> poisVisited;

  // for (const auto &p : points) {
  for (int i = 0; i < points.size();
       i++) {  // i don't check the last points becasue the goal is too close to
               // the boundaries
    const auto &p = points.at(i);
    if (!isPointValid(p.x, p.y, robotRadius)) return false;
  }

  return true;
}

bool PathFinder::pathVisitsAllPOIs(vector<Point3f> points) {
  vector<bool> poisVisited;
  for (auto POI : arena.POIs) {
    poisVisited.push_back(false);
  }
  for (const auto &p : points) {
    for (int i = 0; i < arena.POIs.size(); i++) {
      const float dist = norm(arena.POIs.at(i).position - p);
      // printf("dist : %f \n", dist);
      if (dist < 50) {
        poisVisited.at(i) = true;
      }
    }
  }

  cout << "VISITED: ";
  for (auto i = poisVisited.begin(); i != poisVisited.end(); ++i)
    std::cout << (*i ? 1 : 0) << ' ';
  cout << endl;

  for (bool visited : poisVisited) {
    if (!visited) return false;
  }

  return true;
}

void PathFinder::createNodes(float distance) {
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
        if (distance > (o.radius + safetyDistance)) {  // outside the obstacle
          // forceSum += 1 / (distance - o.radius) *
          // distancePoint;
        } else {  // inside the obstacle
          if (distance < o.innerRadius) {
            ok = false;
            continue;
          } else {
            forceSum = (o.radius - distance + safetyDistance) * distancePoint /
                       cv::norm(distancePoint);
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
bool PathFinder::pointInsideObstacles(const Point &a) {
  for (CircumscribedObstacle c : obstacles) {
    const Point p = c.center;

    if (cv::norm(a - p) < (c.radius)) return true;
  }
  return false;
}
void PathFinder::createSmartNodes() {
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

void PathFinder::createEdges(float distance) {
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

void PathFinder::fromArena(Arena &a) {
  arena = a;
  cout<< "100" <<endl;
  for (auto polygon : arena.obstacles) {
    obstacles.push_back(poligonToCircle(polygon));
  }
  cout<< "200" <<endl;

  const float dist = NODES_DISTANCE;
  // node for each POI
  for (auto poi : arena.POIs) {
    nodes.push_back(GraphNode{Point(poi.position.x, poi.position.y), true});
  }
  cout<< "300" <<endl;

  if (SMART_NODES) {
    createSmartNodes();
    createNodes(dist);
    createEdges(dist * 1.5);

  } else {
    createNodes(dist);
    createEdges(dist * 3.15);
  }
  cout<< "400" <<endl;
}

void PathFinder::drawMapOn(const cv::Mat &image) {
  // circles that contains obstacles
  for (CircumscribedObstacle obstacle : obstacles) {
    circle(image, obstacle.center, obstacle.innerRadius, Scalar(20, 20, 100), 1,
           LINE_AA);
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
vector<GraphNode *> PathFinder::getAStarPath(GraphNode *start,
                                             GraphNode *goal) {
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
  // path.push_back(goal);

  while (n != start) {
    GraphNode *prev = came_from[n];
    path.push_back(prev);
    n = prev;
  }

  reverse(begin(path), end(path));

  path.push_back(goal);
  GraphNode fakenews = GraphNode{Point(300, 300), true};
  // path.push_back(&fakenews);

  return path;
}

void PathFinder::drawPath(Mat image) {
  //  VECTORS
  int i = 0;
  for (auto vv : vectors) {
    for (Point3f vector : vv) {
      const Scalar randColor(rand() * 255, rand() * 255, rand() * 255);
      const auto &a = vector;
      Point2f b(a.x + cos(a.z) * 80, a.y + sin(a.z) * 80);
      line(display, Point(a.x, a.y), b, randColor, 3);
      circle(display, Point(vector.x, vector.y), 4, Scalar(255, 255, 255), 5,
             LINE_AA);

      putText(display, to_string(i++), Point(vector.x, vector.y),
              FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0, 0, 0), 2, CV_AA);
      cv::imshow("Arena parsed", display);
      // cvWaitKey();
    }
  }

  // DUBINS PATH
  for (int i = 0; i < dubinsPath.size(); i++) {
    auto a = dubinsPath[i];
    Point2f b(a.x + cos(a.z) * 5, a.y + sin(a.z) * 5);

    circle(display, Point(a.x, a.y), 1, Scalar(255, 255, 255), 3, LINE_AA);
    line(display, Point(a.x, a.y), b, Scalar(10, 10, 10), 4);

    cv::imshow("Arena parsed", display);
    // cout << "WAIT" << endl;
    // cvWaitKey(15);
  }
}

GraphNode *PathFinder::getClosestNode(Point p) {
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

vector<Point3f> PathFinder::vectorsToDubins(vector<Point3f> const &nodes) {
  vector<Point3f> dubinsPath;

  int start = 0;

  for (int i = 0; i < nodes.size() - 1; i++) {
    const Point3f &v1 = nodes.at(i);
    const Point3f &v2 = nodes.at(i + 1);
    for (auto a : getDubinPath(v1, v2)) dubinsPath.push_back(a);
  }

  /**
      // do {
      //   int end = start + 1;

      //   // trovo un path valido
      //   vector<Point3f> validPath = getDubinPath(nodes[start], nodes[end]);
      //   bool isValid;
      //   do {
      //     validPath = getDubinPath(nodes[start], nodes[end]);
      //     isValid = pathDoesNotCollide(validPath);
      //     if (!isValid) end++;

      //     // cout << "end: " << end << "valid?" << isValid << endl;
      //   } while (!isValid && end < nodes.size());

      //   int validEnd = end;
      //   // vado avanti finchè è valido
      //   vector<Point3f> attemptedPath;
      //   do {
      //     attemptedPath = getDubinPath(nodes[start], nodes[end]);

      //     isValid = pathDoesNotCollide(attemptedPath);
      //     if (isValid) {
      //       validPath = attemptedPath;
      //       validEnd = end;
      //       end++;
      //     }
      //   } while (isValid && end < nodes.size() && (end - start) < 99);

      //   cout << "from : " << start << "   to:  " << validEnd << endl;

      //   for (auto a : validPath) dubinsPath.push_back(a);

      //   start = validEnd;

      // } while (start < nodes.size() - 1);
  */
  return dubinsPath;
}

vector<Point3f> PathFinder::simplify(vector<Point3f> vectors) {
  int index = rand() % (vectors.size() - 2);

  int start = index;
  // int end = start + 1;
  int end = start + 2 + rand() % (vectors.size() - start - 2);

  end = start + 2;

  // int tries = 0;

  // trovo un path valido
  vector<Point3f> attempt = getDubinPath(vectors[start], vectors[end]);
  bool isValid;
  // do {
  // tries++;
  /**
    // if (start > 0 && end < vectors.size() - 1) {
    //   attempt = getDubinPath(vectors[start - 1], vectors[end + 1]);
    //   if (pathDoesNotCollide(attempt)) {
    //     start--;
    //     end++;
    //   }
    // } else {
    //   if (start > 0) {
    //     attempt = getDubinPath(vectors[start - 1], vectors[end]);
    //     if (pathDoesNotCollide(attempt)) {
    //       start--;
    //     }
    //   }

    //   if (end < vectors.size() - 1) {
    //     attempt = getDubinPath(vectors[start], vectors[end + 1]);
    //     if (pathDoesNotCollide(attempt)) {
    //       end++;
    //     }
    //   }
    // } */
  printf("------------\n ");
  printf("attempting from %i to %i , size? %i \n", start, end, vectors.size());

  attempt = getDubinPath(vectors[start], vectors[end]);

  isValid = pathDoesNotCollide(attempt);

  printf("attempting from %i to %i , valid? %i \n", start, end,
         isValid ? 1 : 0);

  // } while (!isValid && (end < nodes.size() || start > 0) && tries < 2);

  // cout << "from : " << start << "   to:  " << validEnd << endl;
  if (isValid) {
    auto copy = vectors;

    cout << "path is valid, trying to erase, " << start << " -> " << end
         << endl;

    const int a = start + 1;
    const int b = end;
    printf("vectors.erase(vectors.begin() + %i, vectors.begin() + %i);", a, b);
    vectors.erase(vectors.begin() + a, vectors.begin() + b);

    cout << "erased, now vectors.size() :" << vectors.size() << endl;

    if (!pathVisitsAllPOIs(vectors)) {
      cout << "NOT VISITING EVRYTHING" << endl;
      vectors = copy;
    } else {
      cout << "YES  ------ VISITING EVRYTHING" << endl;
    }
  }

  return vectors;
}

GraphNode *PathFinder::closestNodeToPOI(POI const &poi) {
  Point point = Point(poi.position.x, poi.position.y);
  return getClosestNode(point);
}

vector<Point3f> PathFinder::nodesToVectorPath(vector<GraphNode *> nodes) {
  vector<Point3f> vectors;
  cout << ">>>: nodes " << nodes.size() << endl;

  float theta;

  for (int i = 0; i < nodes.size() - 1; i++) {
    auto &a = nodes[i]->center;
    auto &b = nodes[i + 1]->center;

    theta = point2angle(b - a);

    Point3f p(a.x, a.y, theta);
    vectors.push_back(p);
  }
  const auto &last = nodes[nodes.size() - 1]->center;
  vectors.push_back(Point3f(last.x, last.y, theta));

  cout << ">>>: vectors " << vectors.size() << endl;

  return vectors;
}

vector<Point3f> PathFinder::testClick(int x, int y, float direction) {
  cout << x << "  " << y << endl;
  // const Point a(300, 350);
  GraphNode *start = getClosestNode(Point(x, y));

  // vector<GraphNode *> totalNodesPath =
  const auto startToFirstPOI =
      getAStarPath(start, closestNodeToPOI(arena.POIs.at(0)));

  // for (int i = 0; i < startToFirstPOI.size() - 1; i++) {
  //   auto a = startToFirstPOI.at(i)->center;
  //   auto b = startToFirstPOI.at(i + 1)->center;

  //   line(display, a, b, Scalar(255, 255, 255), 3);

  //   cv::imshow("Arena parsed", display);
  //   cvWaitKey();
  // }

  vector<vector<GraphNode *>> segments;
  segments.push_back(startToFirstPOI);
  for (int i = 0; i < arena.POIs.size() - 1; i++) {
    GraphNode *a = closestNodeToPOI(arena.POIs.at(i));
    GraphNode *b = closestNodeToPOI(arena.POIs.at(i + 1));
    cout << "getting path from " << arena.POIs.at(i).c << " to  "
         << arena.POIs.at(i + 1).c << endl;
    const auto partialPath = getAStarPath(a, b);

    segments.push_back(partialPath);
    // totalNodesPath.insert(totalNodesPath.end(), partialPath.begin(),
    //                       partialPath.end());
  }

  // add goal
  if (arena.goal.size() > 0) {
    cout << "ADDING GOAL TO PATH" << endl;
    Point center;
    for (auto p : arena.goal) {
      center += p;
    }
    center = center / static_cast<int>(arena.goal.size());

    const auto partial =
        getAStarPath(closestNodeToPOI(arena.POIs.at(arena.POIs.size() - 1)),
                     getClosestNode(center));

    segments.push_back(partial);
  }

  cout << "PATHs DONE" << endl;
  //"vectors" = points with directions

  cout << "generating vectors from nodes" << endl;

  for (auto segment : segments) {
    vectors.push_back(nodesToVectorPath(segment));
  }
  // vectors = nodesToVectorPath(totalNodesPath);

  cout << "generating dubins" << endl;
  vector<Point3f> vectorFlattened;

  for (auto s : vectors) {
    vectorFlattened.insert(vectorFlattened.end(), s.begin(), s.end());
  }

  vectorFlattened.at(0).z = direction;

  dubinsPath = vectorsToDubins(vectorFlattened);

  cv::imshow("Arena parsed", display);
  waitKey(1);

  int tries = 0;
  while (!pathDoesNotCollide(dubinsPath) || tries < 50) {
    tries++;
    cout << "simplify" << endl;
    vectorFlattened = simplify(vectorFlattened);

    display = Scalar(15, 15, 15);
    arena.drawMapOn(display);
    drawMapOn(display);
    drawPath(display);
    for (auto vector : vectorFlattened) {
      const Scalar randColor(rand() * 255, rand() * 255, rand() * 255);
      const auto &a = vector;
      Point2f b(a.x + cos(a.z) * 80, a.y + sin(a.z) * 80);
      line(display, Point(a.x, a.y), b, randColor, 3);
      circle(display, Point(vector.x, vector.y), 4, Scalar(255, 255, 255), 5,
             LINE_AA);
    }
    // waitKey();
    cv::imshow("Arena parsed", display);
    waitKey(1);

    dubinsPath = vectorsToDubins(vectorFlattened);
  }

  display = Scalar(15, 15, 15);
  arena.drawMapOn(display);
  // drawMapOn(display);
  drawPath(display);

  // dubinsPath = partialDubins;

  // for (auto v : vectors) {
  //   cout << "----------------------------------" << endl;

  //   vector<Point3f> partialDubins = vectorsToDubins(v);

  //   while (!pathDoesNotCollide(partialDubins)) {
  //     cout << "simplify" << endl;
  //     v = simplify(v);

  //     display = Scalar(15, 15, 15);
  //     arena.drawMapOn(display);
  //     drawMapOn(display);
  //     for (auto vector : v) {
  //       const Scalar randColor(rand() * 255, rand() * 255, rand() * 255);
  //       const auto &a = vector;
  //       Point2f b(a.x + cos(a.z) * 80, a.y + sin(a.z) * 80);
  //       line(display, Point(a.x, a.y), b, randColor, 3);
  //       circle(display, Point(vector.x, vector.y), 4, Scalar(255, 255,
  //       255),
  //              5, LINE_AA);
  //     }

  //     cv::imshow("Arena parsed", display);
  //     waitKey();

  //     partialDubins = vectorsToDubins(v);
  //   }

  //   dubinsPath.insert(dubinsPath.end(), partialDubins.begin(),
  //                     partialDubins.end());
  // }

  cout << "done with dubins" << endl;

  return dubinsPath;
}

// thanks to http://paulbourke.net/geometry/circlesphere/
bool PathFinder::lineIntersectsWithObstacles(const Point &a, const Point &b) {
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
