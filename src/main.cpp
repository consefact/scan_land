#include "color_detection.h"
#include "image_pre_process.h"

cv::Size size(1080, 720);

void video(std::string path, std::string color_name){
    cv::VideoCapture cap(path);
    cv::Mat image, processed, output;

    process_init(size.width, size.height);
    ColorRanges = color_detect_init();
    while(true){
        cap.read(image);

        process(image, processed);
        ColorResult detect_info = color_detect(processed, color_name);
        draw_info(image, detect_info);

        cv::imshow(color_name + " detect", image);
        // 按ESC退出
        char key = cv::waitKey(30);
        if (key == 27) break;

    }
}

void pic(std::string path, std::string color_name){
    cv::Mat image, processed, output;
    cv::imread(path, image);
    process_init(size.width, size.height);
    ColorRanges = color_detect_init();
    process(image, processed);
    ColorResult detect_info = color_detect(processed, color_name);
    draw_info(image, detect_info);

    cv::imshow(color_name + " detect", image);
    cv::waitKey(0);
}

void wbcam(std::string color_name){
    cv::VideoCapture cap(0);
    cv::Mat image, processed, output;

    process_init(size.width, size.height);
    ColorRanges = color_detect_init();
    while(true){
        cap.read(image);

        process(image, processed);
        ColorResult detect_info = color_detect(processed, color_name);
        draw_info(image, detect_info);

        cv::imshow(color_name + " detect", image);
        // 按ESC退出
        char key = cv::waitKey(30);
        if (key == 27) break;

    }
}

int main(){
    std::cout << "choose mode" << std::endl;
    std::cout << "1. video" << std::endl;
    std::cout << "2. pictrue" << std::endl;
    std::cout << "3. wbcam" << std::endl;
    int chioce;
    std::cin >> chioce;

    std::cout << "choose color which need detect \nin Red Green Blue Yellow Orange Purple Cyan" << std::endl;
    std::string color_name;
    std::cin >> color_name;

    std::string path;
    if(chioce == 1){
        std::cout << "enter path" << std::endl;
        std::cin >> path;
        video(path, color_name);
    } else if (chioce == 2){
        std::cout << "enter path" << std::endl;
        std::cin >> path;
        pic(path, color_name);
    } else if (chioce == 3){
        wbcam(color_name);
    }
    
}
