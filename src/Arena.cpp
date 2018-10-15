
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

class Arena
{
  private:
    int width = 500;
    int height = 750;

    cv::Scalar getColor() //get "random" color to draw
    {
        static int i = 0;
        cv::Scalar colors[] = {cv::Scalar(200, 10, 10), cv::Scalar(0, 200, 0), cv::Scalar(0, 0, 200)};

        return colors[i++ % 3];
    }

  public:
    cv::Mat topView;

    void parseImage(cv::Mat input)
    {
        getTopView(input);
    }

    void getTopView(cv::Mat input, bool debugView = false)
    {
        cv::Mat hsv_img, black_mask;
        cv::cvtColor(input, hsv_img, cv::COLOR_BGR2HSV);

        cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 50), black_mask);

        // cv::imshow("BLACK_filter", black_mask);

        // Find contours
        std::vector<std::vector<cv::Point>> contours, approximation;
        std::vector<cv::Point> arena, arena_approx, largest, largest_approx;
        std::vector<cv::Point> approx_curve;
        cv::Mat contours_img;

        // Apply some filtering
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                                   cv::Size((4 * 2) + 1, (4 * 2) + 1));
        cv::dilate(black_mask, black_mask, kernel);
        cv::erode(black_mask, black_mask, kernel);

        // Process black mask
        cv::findContours(black_mask, contours, cv::RETR_LIST,
                         cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob
        contours_img = input.clone();
        // drawContours(contours_img, contours, -1, cv::Scalar(40, 190, 40), 1, cv::LINE_AA);
        std::cout << "N. contours: " << contours.size() << std::endl;

        double maxArea = 0;
        double arenaArea = 0;

        int epsilon = 30;
        int maxEdges = 8;

        //i take the second biggest polygon, (the biggest is the outer contour)
        for (int i = 0; i < contours.size(); ++i)
        {
            approxPolyDP(contours[i], approx_curve, epsilon, true);
            approximation = {approx_curve};

            if (debugView)
                drawContours(contours_img, approximation, -1, getColor(), 2, cv::LINE_AA);

            double area = contourArea(contours[i]);

            if (area > maxArea && approximation[0].size() < maxEdges)
            {
                arenaArea = area;
                arena_approx = largest_approx;
                arena = largest;

                maxArea = area;
                largest_approx = approx_curve;
                largest = contours[i];
            }
        }
        cout << "arena_approx.size: " << arena_approx.size() << endl;

        // in case it has more than 4 edges for some reason, i reduce it to 4 edges,
        // it should happen only with the outer border but i'll keep it here for safety
        int eps = epsilon;
        while (arena_approx.size() > 4)
        {
            approxPolyDP(arena_approx, arena_approx, eps, true);
            cout << "epsilon to : -> " << eps << endl;
            cout << "reduced to : -> " << arena_approx.size() << endl;
            eps *= 1.10;
        }

        std::vector<std::vector<cv::Point>> aa = {arena_approx};
        std::vector<std::vector<cv::Point>> bb = {arena};
        drawContours(contours_img, aa, -1, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        drawContours(contours_img, bb, 0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

        // "flatten" the map to a rectangular image
        std::vector<cv::Point2f> desidered = {
            cv::Point2f(0, 0),
            cv::Point2f(width, 0),
            cv::Point2f(width, height),
            cv::Point2f(0, height),
        };

        vector<Point2f> floats(arena_approx.begin(), arena_approx.end());

        cv::Mat transform = getPerspectiveTransform(floats, desidered);

        cv::warpPerspective(input, topView, transform, cv::Size(width, height));

        return;
    }
};