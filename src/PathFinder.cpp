
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "./DigitOCR.cpp"
#include "./Arena.cpp"
#include "./utils.cpp"

using namespace std;
using namespace cv;

struct CircumscribedObstacle
{
    Point center;
    float innerRadius;
    float radius;
};

class PathFinder
{
  private:
    CircumscribedObstacle
    poligonToCircle(vector<cv::Point> points)
    {
        cv::Point2f center(0.0f, 0.0f);
        for (auto p : points)
        {
            center.x += p.x;
            center.y += p.y;
        }

        center.x = center.x / points.size();
        center.y = center.y / points.size();

        float maxRadius = 0;
        for (Point2f p : points)
        {
            float radius = cv::norm(center - p);
            if (radius > maxRadius)
                maxRadius = radius;
        }

        return CircumscribedObstacle{center, maxRadius, maxRadius + 15};
    }

    std::vector<CircumscribedObstacle> POIs;

  public:
    Arena arena;
    vector<CircumscribedObstacle> obstacles;

    PathFinder(Arena a)
    {
        arena = a;

        for (auto polygon : arena.obstacles)
        {
            obstacles.push_back(poligonToCircle(polygon));
        }
    }

    void drawMapOn(cv::Mat &image)
    {
        //circles that contains obstacles
        for (CircumscribedObstacle
                 obstacle : obstacles)
        {
            circle(image, obstacle.center, obstacle.innerRadius, Scalar(20, 20, 100), 1, LINE_AA);
            circle(image, obstacle.center, obstacle.radius, Scalar(20, 20, 255), 2, LINE_AA);
        }
    }
};