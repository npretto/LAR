
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
    cv::Mat topView; //clean top-view, as in input
    cv::Mat topView_hsv;

    cv::Mat topViewAnnotated; //top-view with stuff on it

    std::vector<std::vector<cv::Point>> obstacles;
    std::vector<cv::Point> goal;

    void parseImage(cv::Mat input, bool display = false)
    {

        getTopView(input);

        findObstacles(display);
        findGoal(display /*|| true*/);
    }

    void dilateErode(cv::Mat &image)
    {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                                   cv::Size((4 * 2) + 1, (4 * 2) + 1));
        cv::dilate(image, image, kernel);
        cv::erode(image, image, kernel);
    }

    void findObstacles(bool display = false)
    {
        obstacles.clear();

        cv::Mat red_mask;
        cv::cvtColor(topView, topView_hsv, cv::COLOR_BGR2HSV);

        cv::inRange(topView_hsv, cv::Scalar(181 - 55, 20, 20), cv::Scalar(255, 255, 255), red_mask);
        if (display)
            cv::imshow("red_mask", red_mask);
        dilateErode(red_mask);

        std::vector<std::vector<cv::Point>> contours, approximation;

        cv::findContours(red_mask, contours, cv::RETR_LIST,
                         cv::CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); ++i)
        {
            std::vector<cv::Point> approx_curve;

            approxPolyDP(contours[i], approx_curve, 5, true);
            approximation = {approx_curve};

            obstacles.push_back(approx_curve);
        }

        drawContours(topViewAnnotated, obstacles, -1, cv::Scalar(255, 255, 255), 3, cv::LINE_AA);

        if (display)
        {
            cv::imshow("red_mask_eroded", red_mask);
        }
    }

    void findGoal(bool display = false)
    {

        cv::Mat blue_mask;

        cv::inRange(topView_hsv, cv::Scalar(140 - 55, 100, 50), cv::Scalar(140 + 25, 255, 255), blue_mask);

        if (display)
            cv::imshow("blue_mask", blue_mask);

        dilateErode(blue_mask);

        std::vector<std::vector<cv::Point>> contours, approximation;

        cv::findContours(blue_mask, contours, cv::RETR_LIST,
                         cv::CHAIN_APPROX_SIMPLE);

        // drawContours(topViewAnnotated, contours, 0, cv::Scalar(255, 20, 20), 3, cv::LINE_AA);

        std::vector<cv::Point> approx_curve;

        for (int i = 0; i < contours.size(); ++i)
        {
            approxPolyDP(contours[i], approx_curve, 20, true);
            approximation = {approx_curve};

            // if (display)
            // drawContours(contours_img, approximation, -1, getColor(), 2, cv::LINE_AA);

            double area = contourArea(contours[i]);

            if (area > 40)
            {
                goal = approx_curve;
            }
        }

        std::vector<std::vector<cv::Point>> a = {approx_curve};

        drawContours(topViewAnnotated, a, -1, cv::Scalar(20, 20, 255), 2, cv::LINE_AA);

        if (display)
            cv::imshow("blue_mask_eroded", blue_mask);
    }

    void getTopView(cv::Mat input, bool debugView = false)
    {
        cv::Mat topView_hsv, black_mask;
        cv::cvtColor(input, topView_hsv, cv::COLOR_BGR2HSV);

        cv::inRange(topView_hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 50), black_mask);

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

        //detect if the image is "rotated"
        if (cv::norm(arena_approx.at(0) - arena_approx.at(1)) > cv::norm(arena_approx.at(1) - arena_approx.at(2)))
        {
            cout << "YES" << endl;
            desidered.push_back(desidered.at(0));
            desidered.erase(desidered.begin());
        }

        vector<Point2f> floats(arena_approx.begin(), arena_approx.end());

        cv::Mat transform = getPerspectiveTransform(floats, desidered);

        cv::warpPerspective(input, topView, transform, cv::Size(width, height));

        topViewAnnotated = topView.clone();

        cv::cvtColor(topView, topView_hsv, cv::COLOR_BGR2HSV);

        return;
    }
};