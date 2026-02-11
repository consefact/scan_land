// clang-format Language: Cpp
#ifndef ROS_INTER_H
#define ROS_INTER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// 使用 mutex 保护共享数据
#include <mutex>

extern cv::Mat image_origin;
extern std::mutex image_mutex;  // 添加互斥锁

void image_convrt(image_transport::ImageTransport& it_);

/**
 * @brief convert ros image msg to opencv format
 * @param msg image msg from ros
 */
void image_callback(const sensor_msgs::ImageConstPtr& msg);

#endif