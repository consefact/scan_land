#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PointStamped.h>
#include <vision_msgs/Detection2DArray.h>
#include <image_transport/image_transport.h>
#include <mutex>
#include <atomic>

#include "color_detection.h"
#include "image_pre_process.h"
#include "ros_inter.h"

// 全局数据（需要线程安全）
namespace {
    // 任务模式
    std::atomic<int> mission_num{0};

    // YOLO检测结果
    vision_msgs::Detection2DArray latest_detections;
    cv::Rect roi;                       // 当前感兴趣区域
    cv::Mat image_roi;                   // ROI图像（从image_origin裁剪）
    std::mutex yolo_mutex;               // 保护latest_detections, roi, image_roi

    cv::Mat image_processed;

    // 颜色检测结果缓存
    ColorResult takeoff_color;
    ColorResult land_color;
    std_msgs::String takeoff_msg;
    std_msgs::String land_color_msg;
    std_msgs::Bool land_detected_msg;
    std_msgs::Int8 mission_msg;
}

// YOLO检测框回调
void yolo_boxes_cb(const vision_msgs::Detection2DArray::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(yolo_mutex);
    latest_detections = *msg;

    if (!latest_detections.detections.empty()) {
        const auto& det = latest_detections.detections[0];
        // 计算边界框（确保不越界）
        int x = cvRound(det.bbox.center.x - det.bbox.size_x / 2.0);
        int y = cvRound(det.bbox.center.y - det.bbox.size_y / 2.0);
        int w = cvRound(det.bbox.size_x);
        int h = cvRound(det.bbox.size_y);

        // 裁剪到图像范围内
        if (!image_origin.empty()) {
            x = std::max(0, std::min(x, image_origin.cols - 1));
            y = std::max(0, std::min(y, image_origin.rows - 1));
            w = std::min(w, image_origin.cols - x);
            h = std::min(h, image_origin.rows - y);
        }

        roi = cv::Rect(x, y, w, h);
        if (w > 0 && h > 0 && !image_origin.empty()) {
            image_roi = image_origin(roi).clone();  // 深拷贝，避免后续修改原始图像影响
        } else {
            image_roi = cv::Mat();  // 清空无效ROI
        }
    } else {
        // 无检测框时，清空ROI
        roi = cv::Rect();
        image_roi = cv::Mat();
    }
}

// 任务指令回调
void mission_cb(const std_msgs::Int8::ConstPtr &msg) {
    mission_num.store(msg->data);
}

int main(int argc, char **argv) {
    // 防止中文输出乱码
    setlocale(LC_ALL, "");

    // 初始化ROS节点
    ros::init(argc, argv, "scan_land");
    ros::NodeHandle nh;

    // 图像传输初始化（订阅原始图像）
    image_transport::ImageTransport it_(nh);
    image_convrt(it_);   // 假设该函数订阅了原始图像，并更新全局image_origin

    // 初始化颜色检测参数
    ColorRanges = color_detect_init();
    process_init();      // 图像预处理初始化

    // 订阅话题
    ros::Subscriber mission_sub = nh.subscribe<std_msgs::Int8>("/color_detect/mission_num", 10, mission_cb);
    ros::Subscriber land_color_sub = nh.subscribe<vision_msgs::Detection2DArray>("/yolo/detection_boxes", 10, yolo_boxes_cb);

    // 发布话题
    ros::Publisher takeoff_color_pub = nh.advertise<std_msgs::String>("/color_detect/takeoff_color", 10);
    ros::Publisher land_color_pub = nh.advertise<std_msgs::String>("/color_detect/land_color", 10);
    ros::Publisher land_detected_pub = nh.advertise<std_msgs::Bool>("/color_detect/land_detected", 10);

    // 循环频率（20Hz，大于2Hz以满足飞控心跳）
    ros::Rate rate(20);

    // 主循环
    int n = 0;
    while (ros::ok()) {
        int current_mission = mission_num.load();

        switch (current_mission) {
            case 0:
                if (n % 5 == 0) {
                    ROS_INFO("等待任务指令...");
                }
                n = (n + 1) % 5;
                break;

            case 1: {
                ROS_INFO("执行任务 1：检测起飞颜色");
                cv::Mat local_roi;
                {
                    std::lock_guard<std::mutex> lock(yolo_mutex);
                    if (!image_roi.empty()) {
                        local_roi = image_roi.clone();  // 拷贝到局部变量，避免长时间持有锁
                    }
                }

                if (local_roi.empty()) {
                    ROS_WARN_THROTTLE(1, "ROI图像为空，等待YOLO检测框...");
                    break;
                }

                if (process(local_roi, image_processed)) {
                    takeoff_color = color_detect(image_processed);
                    if (!takeoff_color.is_detected) {
                        ROS_WARN_THROTTLE(1, "未检测到任何颜色");
                        break;
                    }
                    takeoff_msg.data = takeoff_color.name;
                    ROS_INFO("检测到起飞颜色: %s", takeoff_color.name.c_str());
                    takeoff_color_pub.publish(takeoff_msg);
                    mission_num.store(0);   // 完成后回到等待状态
                }
                break;
            }

            case 2: {
                ROS_INFO("执行任务 2：检测降落颜色");
                cv::Mat local_roi;
                {
                    std::lock_guard<std::mutex> lock(yolo_mutex);
                    if (!image_roi.empty()) {
                        local_roi = image_roi.clone();
                    }
                }

                if (local_roi.empty()) {
                    ROS_WARN_THROTTLE(1, "ROI图像为空，等待YOLO检测框...");
                    break;
                }

                if (process(local_roi, image_processed)) {
                    land_color = color_detect(image_processed, takeoff_color.name);
                    ROS_INFO("降落颜色: %s, 检测到: %d",
                             land_color.name.c_str(), land_color.is_detected);

                    land_color_msg.data = land_color.name;
                    land_detected_msg.data = land_color.is_detected;

                    land_color_pub.publish(land_color_msg);
                    land_detected_pub.publish(land_detected_msg);
                }
                break;
            }

            case 3:
                ROS_INFO("任务完成，节点退出");
                ros::shutdown();
                return 0;

            default:
                ROS_WARN("未知任务编号: %d", current_mission);
                mission_num.store(0);
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}