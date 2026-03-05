#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PointStamped.h>
#include "color_detection.h"
#include "image_pre_process.h"
#include "ros_inter.h"
#include <vision_msgs/Detection2DArray.h>

int mission_num = 0;
cv::Mat image_processed;
ColorResult takeoff_color;
ColorResult land_color;
std_msgs::String takeoff_msg;
std_msgs::String land_color_msg;
std_msgs::Bool land_detected_msg;
std_msgs::Int8 mission_msg;
vision_msgs::Detection2DArray latest_detections;
cv::Mat image_roi;
cv::Rect roi;

void yolo_boxes_cb(const vision_msgs::Detection2DArray::ConstPtr &msg) {
    latest_detections = *msg;
    // 可以进一步处理，例如找出最大框、特定类别等

}

void mission_cb(const std_msgs::Int8::ConstPtr &msg){
    mission_msg = *msg;
    mission_num = mission_msg.data;
}

int main(int argc, char **argv)
{
    // 防止中文输出乱码
    setlocale(LC_ALL, "");

    // 初始化ROS节点
    ros::init(argc, argv, "scan_land");
    ros::NodeHandle nh;

    image_transport::ImageTransport it_(nh);
    image_convrt(it_);
    ColorRanges = color_detect_init();
    process_init();

    //mission_sub
    ros::Subscriber mission_sub = nh.subscribe<std_msgs::Int8>("/color_detect/mission_num", 10, mission_cb);

    //takeoff color pub
    ros::Publisher takeoff_color_pub = nh.advertise<std_msgs::String>("/color_detect/takeoff_color", 10);

    //land color pub
    ros::Publisher land_color_pub = nh.advertise<std_msgs::String>("/color_detect/land_color", 10);
    ros::Publisher land_detected_pub = nh.advertise<std_msgs::Bool>("/color_detect/land_detected", 10);
    //land color sub
    ros::Subscriber land_color_sub = nh.subscribe<vision_msgs::Detection2DArray>("/yolo/detection_boxes", 10, yolo_boxes_cb);
    // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
    ros::Rate rate(20);

    ros::spinOnce();
    rate.sleep();

    int n = 0;
    while(ros::ok()){
        switch(mission_num){
            case 0:
                if(!(n%5)){
                    n %= 5;
                    ROS_INFO("waiting");
                }
                n++;
                break;
            case 1:
                ROS_INFO("mission 1");
                if(process(image_origin, image_processed)){
                    takeoff_color = color_detect(image_processed);
                    if(!takeoff_color.is_detected){
                        ROS_WARN("no color was detected");
                        break;
                    }
                    takeoff_msg.data = takeoff_color.name;
                    ROS_INFO("color was detected: %s", takeoff_color.name);
                    takeoff_color_pub.publish(takeoff_msg);
                    mission_num = 0;
                }
                break;
            case 2: 
                ROS_INFO("mission 2");
                if(!latest_detections.detections.empty()){
                    const auto& det = latest_detections.detections[0];
                    roi.x = cvRound(det.bbox.center.x - det.bbox.size_x / 2.0);
                    roi.y = cvRound(det.bbox.center.y - det.bbox.size_y / 2.0);
                    roi.width = cvRound(det.bbox.size_x);
                    roi.height = cvRound(det.bbox.size_y);
                }
                image_roi = image_origin(roi);
                if(process(image_roi, image_processed)){
                    land_color = color_detect(image_processed, takeoff_color.name);
                    ROS_INFO("land_color: %s, is_detected: %d", land_color.name.c_str(), land_color.is_detected);
                    land_color_msg.data = land_color.name;
                    land_detected_msg.data = land_color.is_detected;
                    land_color_pub.publish(land_color_msg);
                    land_detected_pub.publish(land_detected_msg);
                }
                break;
            case 3:
                ROS_INFO("mission complete");
                return 0;
        }
        ros::spinOnce();
        rate.sleep();
    }
}