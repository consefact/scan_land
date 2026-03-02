// clang-format Language: Cpp
#ifndef YOLO_DETECTOR_H
#define YOLO_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <mutex>

class YOLODetector {
public:
    YOLODetector(ros::NodeHandle& nh);
    ~YOLODetector();
    
    bool initialize();
    void run();

    bool getResult_detected();
    geometry_msgs::Point getResult_info();

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void loadModel(const std::string& model_path);
    std::vector<cv::Rect> detect(const cv::Mat& image);
    void publishResults(const std::vector<cv::Rect>& detections, 
                       const ros::Time& stamp);
private:
    ros::NodeHandle nh_;
    
    // ROS通信
    image_transport::Subscriber image_sub_;
    ros::Publisher detection_pub_;
    image_transport::Publisher debug_image_pub_;
    
    // 模型相关
    cv::dnn::Net net_;
    std::vector<std::string> classes_;
    
    // 参数
    std::string model_path_;
    float confidence_threshold_;
    float nms_threshold_;
    int input_width_;
    int input_height_;
    int num_classes_from_config;
    // double last_process_time;
    // int frame_count;

    //yolo detect result
    bool target_detected;
    geometry_msgs::Point target_info;
};

#endif // YOLO_DETECTOR_H