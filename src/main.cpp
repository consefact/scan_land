#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

int main(int argc, char* argv[]) {
    std::string path = argv[1];
    cv::Mat img = cv::imread(path);
    cv::Mat img_hsv, mask;

    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

    int hmin = 156, hmax = 180, smin = 43, smax = 255, vmin = 46, vmax = 255;
    cv::namedWindow("track bar", (400,400));
    cv::createTrackbar("hmin", "track bar", &hmin, 179);
    cv::createTrackbar("hmax", "track bar", &hmax, 179);
    cv::createTrackbar("smin", "track bar", &smin, 255);
    cv::createTrackbar("smax", "track bar", &smax, 255);
    cv::createTrackbar("vmin", "track bar", &vmin, 255);
    cv::createTrackbar("vmax", "track bar", &vmax, 255);

    while(1){
    cv::Scalar lower(hmin, smin, vmin), upper(hmax, smax, vmax);
    cv::inRange(img_hsv, lower, upper, mask);

    cv::imshow("img", img);
    cv::imshow("img_hsv", img_hsv);
    cv::imshow("mask", mask);
    cv::waitKey(1);
    }
    
}
