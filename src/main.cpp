#include <opencv2/opencv.hpp>

int main(){
    cv::VideoCapture cap(0);
    cv::Mat img;
    while(true){
        cap.read(img);
        cv::imshow("test", img);
        cv::waitKey(1);
    }
}
