#ifndef MAIN_H
#define MAIN_H

using namespace std;
using namespace cv;

#include <tesseract/baseapi.h>  //this has to stay on top otherwise the console will be full of errors :(
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <string>

extern bool STOP_AT_EVERY_OCR;
extern bool DRAW_VISITED_EDGES;
extern bool DRAW_EDGES;
extern bool SMART_NODES;

extern float NODES_DISTANCE;
extern float safetyDistance;
extern float robotRadius;
extern float TURNING_RADIUS;

extern float MAP_WIDTH;
extern float MAP_HEIGHT;

extern cv::Mat display;

float pixelToCm(float pixels);
float cmToPixels(float cm);

#include "./main.cpp"

#endif