#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

int main(int argc, char* argv[]) {
    std::string path = argv[1];
    cv::Mat img = cv::imread(path);
    cv::Mat img_resize, img_crop;

    cv::resize(img, img_resize, cv::Size(), 0.5, 2);

    cv::Rect roi(100, 100, 300, 200);
    img_crop = img(roi);
    std::cout << img.size() << std::endl;
    cv::imshow("img", img);
    cv::imshow("resize", img_resize);
    cv::imshow("crop", img_crop);
    cv::waitKey(0);
}
