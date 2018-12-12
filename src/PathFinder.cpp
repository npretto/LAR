
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
#include "../main.h"

using namespace std;
using namespace cv;

struct CircumscribedObstacle
{
    Point center;
    float innerRadius;
    float radius;
};

struct GraphNode
{
    Point center;
    bool ok;
};

struct Edge
{
    GraphNode *a;
    GraphNode *b;
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

        return CircumscribedObstacle{center, maxRadius, maxRadius + robotRadius};
    }

    void createNodes(float distance)
    {
        int cols = arena.getWidth() / distance;
        int rows = arena.getHeight() / distance;
        for (int row = 0; row < rows; row++)
        {
            vector<GraphNode> rowVector;
            for (int col = 0; col < cols; col++)
            {
                bool ok = true;
                Point p(col * distance, row * distance);
                //force field degli ostacoli
                Point forceSum(0, 0);
                for (CircumscribedObstacle o : obstacles)
                {
                    Point distancePoint = p - o.center;
                    float distance = cv::norm(distancePoint);
                    if (distance > o.radius) //outside the obstacle
                    {

                        // forceSum += 1 / (distance - o.radius) * distancePoint;
                    }
                    else
                    { //inside the obstacle
                        if (distance < o.innerRadius)
                        {
                            ok = false;
                            continue;
                        }
                        else
                        {
                            forceSum = (o.radius - distance + safetyDistance) * distancePoint / cv::norm(distancePoint);
                        }
                    }
                }

                Point position = ok ? p + forceSum : p;
                GraphNode node{position, ok};
                // rowVector.push_back(node);
                if (ok)
                    nodes.push_back(node);
            }
            // nodesMap.push_back(rowVector);
        }
    }

    void createEdges(float distance)
    {
        for (int i = 0; i < nodes.size(); i++)
        {
            for (int j = i; j < nodes.size(); j++)
            {
                GraphNode *a = &nodes[i];
                GraphNode *b = &nodes[j];

                float dist = cv::norm(a->center - b->center);

                if (dist < distance)
                {
                    edges.push_back(Edge{a, b});
                }
            }
        }
    }

  public:
    Arena arena;
    vector<CircumscribedObstacle> obstacles;
    // vector<vector<GraphNode>> nodesMap;
    vector<GraphNode> nodes;
    vector<Edge> edges;

    PathFinder(Arena a)
    {
        arena = a;

        for (auto polygon : arena.obstacles)
        {
            obstacles.push_back(poligonToCircle(polygon));
        }
        createNodes(30);
        createEdges(40);
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

        //edges
        for (auto e : edges)
        {
            line(image, e.a->center, e.b->center, Scalar(120, 120, 120), 1);
        }

        //nodes
        for (GraphNode node : nodes)
        {
            if (node.ok)
                circle(image, node.center, 2, Scalar(255, 255, 255), 1, LINE_AA);
            else
                circle(image, node.center, 2, Scalar(0, 0, 255), 1, LINE_AA);
        }

    }
};