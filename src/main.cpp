#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

int main(int argc, char* argv[]) {
    std::string path = argv[1];
    cv::Mat img_ori = cv::imread(path), img_gray, img_blur, img_cny, img_dil, img_ero;

    cv::cvtColor(img_ori, img_gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(img_ori, img_blur, cv::Size(3, 3), 3, 0);
    cv::Canny(img_blur, img_cny, 25, 75);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(img_cny, img_dil, kernel);
    cv::erode(img_dil, img_ero, kernel);

    cv::imshow("blur",img_blur);
    cv::imshow("gray", img_gray);
    cv::imshow("canny", img_cny);
    cv::imshow("dilate", img_dil);
    cv::imshow("erode", img_ero);

    cv::waitKey(0);
}
