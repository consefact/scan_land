#include "color_detection.h"

std::vector<ColorRange> color_detect_init(){
    return {
        {"Red", cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), cv::Scalar(0, 0, 255)},
        {"Red", cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), cv::Scalar(0, 0, 255)},
        {"Green", cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), cv::Scalar(0, 255, 0)},
        {"Blue", cv::Scalar(100, 150, 0), cv::Scalar(140, 255, 255), cv::Scalar(255, 0, 0)},
        {"Yellow", cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), cv::Scalar(0, 255, 255)},
        {"Orange", cv::Scalar(10, 100, 100), cv::Scalar(20, 255, 255), cv::Scalar(0, 165, 255)},
        {"Purple", cv::Scalar(140, 50, 50), cv::Scalar(160, 255, 255), cv::Scalar(255, 0, 255)},
        {"Cyan", cv::Scalar(85, 100, 100), cv::Scalar(100, 255, 255), cv::Scalar(255, 255, 0)}
    };
}

ColorResult color_detect(cv::Mat input, std::string color_name){
    cv::Mat mask;
    if(color_name.compare("red")){
        cv::Mat mask1, mask2;
        cv::inRange(input, ColorRanges[0].lower, ColorRanges[0].upper, mask1);
        cv::inRange(input, ColorRanges[1].lower, ColorRanges[1].upper, mask2);
        mask = mask1 | mask2;
    } else {
        for(auto color: ColorRanges){
            if(color.name.compare(color_name)){
                cv::inRange(input, color.lower, color.upper, mask);
            }
        }
    }
    if(mask.empty()){
        std::cerr << "no suitable color for detect" << std::endl;
        return {"error", 0, cv::Rect(0, 0, 0, 0), cv::Point(0,0), false};
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    int pix_counter = 0, center_x = 0, center_y = 0;
    for(int y = 0; y < mask.rows; y++){
        for(int x = 0; x < mask.cols; x++){
            if(mask.data[y*mask.cols + x] == 255){
                center_x += x;
                center_y += y;
                pix_counter++;
            }
        }
    }

    if(pix_counter < 0){
        std::cout << "no target color in input" << std::endl;
        return {"no target color", 0, cv::Rect(0, 0, 0, 0), cv::Point(0, 0), false};
    } 

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> contour_max;
    for(auto contour: contours){
        double area = cv::contourArea(contour), area_max;
        if(area > 500){
            if(area > area_max){
                area_max = area;
                contour_max = contour;
            }
        }
    }

    cv::Rect rect = cv::boundingRect(contour_max);

    return {color_name, static_cast<double>(pix_counter) / (input.cols * input.rows), rect, cv::Point(center_x / input.cols, center_y / input.rows), true};
}

void draw_info(cv::Mat input, cv::Mat output, ColorResult info){
    // 绘制边界矩形
    cv::rectangle(input, info.largest_rect, cv::Scalar(0, 255, 0), 2);
                
    // 添加文本标签
    std::string label = info.name + " (" + std::to_string(info.percentage) + ")";
    putText(input, label, cv::Point(info.largest_rect.x, info.largest_rect.y - 10),cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

}