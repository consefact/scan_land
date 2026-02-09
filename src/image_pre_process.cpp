#include "image_pre_process.h"

///全局变量
cv::Size target_size;   //预定图像大小
cv::Ptr<cv::CLAHE> clahe_;  // CLAHE对象
int blur_kernel_size;

void process_init(int target_width, int target_height, const int& kernel_size){
    target_size = cv::Size(target_width, target_height);

    blur_kernel_size = kernel_size;
}

bool process(cv::Mat& input, cv::Mat& output){
    if(input.empty()){
        std::cerr << "the input of image preprocess is empty" << std::endl;
        return false;
    }

    cv::resize(input, input, target_size);
    
    cv::Mat img_clahe;
    applyCLAHE(input, img_clahe);
    
    cv::imshow("clahe", img_clahe);

    cv::Mat img_blur;
    cv::GaussianBlur(img_clahe, img_blur, cv::Size(blur_kernel_size, blur_kernel_size), 0);
    
    cv::imshow("blur", img_blur);

    cv::cvtColor(img_blur, output, cv::COLOR_BGR2HSV);

    return true;
}

void applyCLAHE(const cv::Mat& input, cv::Mat& output){
    //创建CLAHE对象
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE();
    clahe_ ->setClipLimit(2.0);
    clahe_ ->setTilesGridSize(cv::Size(8, 8));

    if(input.channels() == 1) {
        clahe_->apply(input, output);
    } else {
        cv::Mat image_lab;
        cv::cvtColor(input, image_lab, cv::COLOR_BGR2Lab);

        std::vector<cv::Mat> channels_lab;
        cv::split(image_lab, channels_lab);

        clahe_->apply(channels_lab[0], channels_lab[0]);

        cv::merge(channels_lab, image_lab);
        cv::cvtColor(image_lab, output, cv::COLOR_Lab2BGR);
    }
}