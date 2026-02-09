#ifndef IMAGE_PRE_PROCESS_H
#define IMAGE_PRE_PROCESS_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

///全局变量
cv::Size target_size;   //预定图像大小
cv::Ptr<cv::CLAHE> clahe_;  // CLAHE对象
int blur_kernel_size;
///主要函数

/**
 * @brief 预处理初始化
 * @param target_width 预定图像宽度
 * @param target_height 预定图像长度
 * @param kernel_size 高斯模糊核大小(为奇数)
 */
void process_init(int target_width = 640, int target_height = 480, const int& kernel_size = 5);

/**
 * @brief 处理图像的主要函数
 * @param input 输入图像 (BGR格式)
 * @param output 输出图像 (HSV格式)
 * @return 处理成功返回true
 */
bool process(const cv::Mat& input, cv::Mat& output);

/**
 * @brief CLAHE处理函数
 * @param input 输入图像 (BGR or GRAY)
 * @param output 输出图像
 */
void applyCLAHE(const cv::Mat& image, cv::Mat& output);

#endif