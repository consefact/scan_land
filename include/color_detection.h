#ifndef COLOR_DETECTION_H
#define COLOR_DETECTION_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

/**
 * @brief 颜色范围定义
 */
struct ColorRange{
    std::string name;
    cv::Scalar lower;
    cv::Scalar upper;
    cv::Scalar displayColor;
};

/**
 * @brief 颜色检测结果
 */
struct ColorResult{
    std::string name;
    double percentage;
    cv::Rect largest_rect;
    cv::Point central_point;
    bool is_detected;
};

//储存颜色范围
extern std::vector<ColorRange> ColorRanges;

/**
 * @brief 颜色检测初始化(初始化颜色范围)
 * @return 返回预设颜色范围
 */
std::vector<ColorRange> color_detect_init();

/**
 * @brief 颜色检测主体
 * @param input 图像输入(HSV)(经过预处理)
 * @param color_name 需检测颜色名
 * @return 返回检测结果
 */
ColorResult color_detect(cv::Mat input, std::string color_name);

/**
 * @brief 识别结果标识(边框，中心，颜色)
 * @param 受标识图像(BGR)
 * @param 输出图像
 * @param 标识信息(颜色识别结果)
 */
void draw_info(cv::Mat input, ColorResult info);

#endif