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