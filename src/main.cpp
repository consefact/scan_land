#include "color_detection.h"
#include "image_pre_process.h"

int main(){
    std::string color_name;
    std::cin >> color_name;

    cv::VideoCapture cap(0);
    cv::Mat image, processed, output;

    process_init();
    ColorRanges = color_detect_init();
    while(true){
        cap.read(image);

        process(image, processed);
        ColorResult detect_info = color_detect(processed, color_name);
        draw_info(image, output, detect_info);

        cv::imshow(color_name + " detect", output);
        // 按ESC退出
        char key = cv::waitKey(30);
        if (key == 27) break;

    }
}
