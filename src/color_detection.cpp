#include "color_detection.h"

// 全局颜色范围列表，使用时若为空则自动初始化
std::vector<ColorRange> ColorRanges;

// 初始化颜色范围（保持原接口）
std::vector<ColorRange> color_detect_init() {
    return {
        {"Red", cv::Scalar(0, 43, 46), cv::Scalar(10, 255, 255), cv::Scalar(0, 0, 255)},
        {"Red", cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), cv::Scalar(0, 0, 255)},
        {"Green", cv::Scalar(35, 43, 46), cv::Scalar(77, 255, 255), cv::Scalar(0, 255, 0)},
        {"Blue", cv::Scalar(100, 43, 46), cv::Scalar(124, 255, 255), cv::Scalar(255, 0, 0)},
        {"Yellow", cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255), cv::Scalar(0, 255, 255)},
        {"Orange", cv::Scalar(11, 43, 46), cv::Scalar(25, 255, 255), cv::Scalar(0, 165, 255)},
        {"Purple", cv::Scalar(125, 43, 46), cv::Scalar(155, 255, 255), cv::Scalar(255, 0, 255)},
        {"Cyan", cv::Scalar(85, 100, 100), cv::Scalar(100, 255, 255), cv::Scalar(255, 255, 0)},
        {"black", cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 46), cv::Scalar(0, 0, 0)}
    };
}

// 辅助函数：计算二值图像的非零像素总数和中心点（图像矩）
static bool compute_mask_stats(const cv::Mat& mask, int& total_pixels, cv::Point& center) {
    total_pixels = cv::countNonZero(mask);
    if (total_pixels == 0) {
        center = cv::Point(0, 0);
        return false;
    }
    // 使用图像矩求中心
    cv::Moments m = cv::moments(mask, true);
    center = cv::Point(static_cast<int>(m.m10 / m.m00), static_cast<int>(m.m01 / m.m00));
    return true;
}

// 辅助函数：对mask进行形态学开闭运算
static void morphology_filter(cv::Mat& mask) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
}

// 主检测函数（接口不变）
ColorResult color_detect(cv::Mat input, std::string color_name) {
    // 若全局颜色列表为空则自动初始化
    if (ColorRanges.empty()) {
        ColorRanges = color_detect_init();
    }

    cv::Mat mask;
    bool detect_mode = false;   // 未使用，可移除，但保留以保持接口兼容性

    // 处理 "None" 模式：找出面积最大的颜色
    if (color_name == "None") {
        int best_index = -1;
        int max_area = 0;
        cv::Mat best_mask;   // 可选：直接保存最佳mask避免二次计算

        for (size_t i = 0; i < ColorRanges.size(); ++i) {
            const auto& color = ColorRanges[i];
            cv::Mat cur_mask;
            if (color.name == "Red") {
                // 红色特殊处理：两个范围
                cv::Mat mask1, mask2;
                cv::inRange(input, ColorRanges[0].lower, ColorRanges[0].upper, mask1);
                cv::inRange(input, ColorRanges[1].lower, ColorRanges[1].upper, mask2);
                cur_mask = mask1 | mask2;
            } else {
                cv::inRange(input, color.lower, color.upper, cur_mask);
            }
            int area = cv::countNonZero(cur_mask);
            if (area > max_area) {
                max_area = area;
                best_index = static_cast<int>(i);
                best_mask = cur_mask.clone();   // 保存最佳mask
            }
        }

        if (best_index == -1 || max_area == 0) {
            return {"no target color", 0.0, cv::Rect(0,0,0,0), cv::Point(0,0), false};
        }

        color_name = ColorRanges[best_index].name;
        mask = best_mask;   // 直接使用已计算的mask
    }
    // 处理 "Red" 特殊模式
    else if (color_name == "Red") {
        cv::Mat mask1, mask2;
        cv::inRange(input, ColorRanges[0].lower, ColorRanges[0].upper, mask1);
        cv::inRange(input, ColorRanges[1].lower, ColorRanges[1].upper, mask2);
        mask = mask1 | mask2;
    }
    // 其他指定颜色
    else {
        bool found = false;
        for (const auto& color : ColorRanges) {
            if (color.name == color_name) {
                cv::inRange(input, color.lower, color.upper, mask);
                found = true;
                break;
            }
        }
        if (!found) {
            std::cerr << "Unknown color name: " << color_name << std::endl;
            return {"error", 0.0, cv::Rect(0,0,0,0), cv::Point(0,0), false};
        }
    }

    // 检查mask是否有效
    if (mask.empty()) {
        std::cerr << "Failed to create mask for color: " << color_name << std::endl;
        return {"error", 0.0, cv::Rect(0,0,0,0), cv::Point(0,0), false};
    }

    // 形态学滤波
    morphology_filter(mask);

    // 计算像素总数和中心点
    int total_pixels = 0;
    cv::Point center;
    if (!compute_mask_stats(mask, total_pixels, center)) {
        std::cout << "No target color in input" << std::endl;
        return {"no target color", 0.0, cv::Rect(0,0,0,0), cv::Point(0,0), false};
    }

    // 查找最大轮廓（面积 > 500）
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double max_contour_area = 0;
    std::vector<cv::Point> largest_contour;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 500 && area > max_contour_area) {
            max_contour_area = area;
            largest_contour = contour;
        }
    }

    cv::Rect rect;
    if (!largest_contour.empty()) {
        rect = cv::boundingRect(largest_contour);
    } else {
        rect = cv::Rect(0, 0, 0, 0);
    }

    // 计算占比
    double percentage = static_cast<double>(total_pixels) / (input.cols * input.rows);

    return {color_name, percentage, rect, center, true};
}

// 绘制信息（接口不变）
void draw_info(cv::Mat input, ColorResult info) {
    if (!info.success || info.largest_rect.area() == 0) return;

    // 绘制边界矩形
    cv::rectangle(input, info.largest_rect, cv::Scalar(0, 255, 0), 2);

    // 格式化百分比（保留两位小数）
    std::ostringstream label_stream;
    label_stream << info.name << " (" << std::fixed << std::setprecision(2) << info.percentage * 100 << "%)";
    std::string label = label_stream.str();

    // 添加文本标签
    cv::putText(input, label,
                cv::Point(info.largest_rect.x, info.largest_rect.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
}